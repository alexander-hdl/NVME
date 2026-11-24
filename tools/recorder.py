#!/usr/bin/env python3
"""recorder.py - PC side of the AX7A200B recorder prototype (STAND/RECORDER_SPEC.md, WP9).

Serial text protocol client (921600 8N1), unit conversion (rate <-> NCO step,
us <-> cycles), reply/telemetry parsing, CSV telemetry log with FPGA power estimate,
fake device for hardware-free use, self-tests.

Python 3 standard library only; pyserial (--port) and matplotlib (plot) are imported lazily.

Examples:
  python tools/recorder.py --selftest
  python tools/recorder.py --fake ping --count 5
  python tools/recorder.py --fake rec --rate 280MB/s --len 4MiB --log run.csv --seconds 2
  python tools/recorder.py --port COM5 prof --file profile.txt --rep 4 --end stop
"""
import argparse
import bisect
import csv
import math
import os
import re
import sys
import time

# ---------------------------------------------------------------------------
# Constants (spec 2.1, 2.3; rec_pkg: UI_CLK_HZ)
# ---------------------------------------------------------------------------
UI_CLK_HZ = 100_000_000
LINK_CLK_NS = 5                 # 5 ns ticks in !R* (link_clk 200 MHz)
SLOT_BYTES = 128 * 1024
BAUD = 921600
MAX_LINE = 64
MAX_SEGMENTS = 64
STEP_MAX = 0xFFFFFFFF
DUR_MAX = 0xFFFFFFFFFF          # h10

MODES = {0: "NONE", 1: "REC", 2: "PF", 3: "READ", 4: "FORMAT", 5: "MON", 6: "DTEST",
         7: "MEMTEST"}
STATES = {0: "IDLE", 1: "BRINGUP", 2: "RUN", 3: "SRC_STOP", 4: "CORE_STOP", 5: "RELEASE",
          6: "LOCAL", 7: "ERROR", 8: "CORE_RST"}
CORE_STATES = {0: "IDLE", 5: "RUN", 6: "DRAIN", 7: "FLUSH", 8: "SHUTDOWN", 9: "DONE",
               15: "ERROR"}
ERR_TEXT = {1: "syntax", 2: "unknown keyword", 3: "argument out of range",
            4: "not allowed in this state", 5: "DDR not calibrated / ui_clk not ready",
            6: "core in error, CORE_RST", 8: "empty profile", 9: "profile full (64)",
            0xA: "line > 64 chars", 0xB: "READ LAST without a recording",
            0xC: "line before previous reply", 0xD: "UART stop-bit error"}


class RecError(Exception):
    pass


class ParseError(RecError):
    pass


class DeviceError(RecError):
    def __init__(self, code, line=""):
        self.code = code
        super().__init__("ERR %02X (%s) %s" % (code, ERR_TEXT.get(code, "?"), line))


# ---------------------------------------------------------------------------
# Unit conversion (spec 2.3)
# ---------------------------------------------------------------------------
def rate_to_step(rate_bps):
    """Bytes/s -> NCO step = round(R * 2^32 / (16 * UI_CLK_HZ))."""
    if rate_bps < 0:
        raise ValueError("negative rate")
    step = int(round(rate_bps * (1 << 32) / (16 * UI_CLK_HZ)))
    if step > STEP_MAX:
        raise ValueError("rate %g B/s exceeds NCO range (max %.6g B/s)"
                         % (rate_bps, step_to_rate(STEP_MAX)))
    return step


def step_to_rate(step):
    """NCO step -> actual rate in bytes/s: step * 16 * UI_CLK_HZ / 2^32."""
    return step * 16 * UI_CLK_HZ / (1 << 32)


def us_to_cycles(us):
    return int(round(us * UI_CLK_HZ / 1e6))


def cycles_to_us(cyc):
    return cyc * 1e6 / UI_CLK_HZ


def bytes_to_slots(nbytes):
    """Bytes -> 128 KiB slots, rounding up."""
    return -(-int(nbytes) // SLOT_BYTES)


def mib_to_ddrsz(mib):
    m = int(math.ceil(mib))
    if not 1 <= m <= 0x400:
        raise ValueError("DDRSZ must be 1..1024 MiB")
    return m


def xadc_temp_c(code):
    return code * 503.975 / 4096 - 273.15


def xadc_volt(code):
    return code * 3.0 / 4096


def temp_c_to_xadc(t):
    return int(round((t + 273.15) * 4096 / 503.975))


def volt_to_xadc(v):
    return int(round(v * 4096 / 3.0))


_RATE_UNITS = {"B/S": 1, "KB/S": 1e3, "MB/S": 1e6, "GB/S": 1e9,
               "KIB/S": 1024, "MIB/S": 1024 ** 2, "GIB/S": 1024 ** 3}
_SIZE_UNITS = {"B": 1, "KB": 1e3, "MB": 1e6, "GB": 1e9,
               "KIB": 1024, "MIB": 1024 ** 2, "GIB": 1024 ** 3}
_TIME_UNITS = {"US": 1e-6, "MS": 1e-3, "S": 1.0, "MIN": 60.0}


def _num_unit(text, units, default, what):
    m = re.fullmatch(r"\s*([0-9]*\.?[0-9]+(?:[eE][+-]?\d+)?)\s*([A-Za-z/]*)\s*", text)
    if not m:
        raise ValueError("bad %s: %r" % (what, text))
    u = m.group(2).upper() or default
    if u not in units:
        raise ValueError("bad %s unit in %r" % (what, text))
    return float(m.group(1)) * units[u]


def parse_rate(text):
    """'280MB/s' -> bytes/s (decimal MB; MiB/s also accepted; bare number = MB/s)."""
    return _num_unit(text, _RATE_UNITS, "MB/S", "rate")


def parse_size(text):
    """'4MiB' -> bytes (bare number = MiB)."""
    return _num_unit(text, _SIZE_UNITS, "MIB", "size")


def parse_duration(text):
    """'30s', '500ms', '10us' -> seconds (bare number = s)."""
    return _num_unit(text, _TIME_UNITS, "S", "duration")


def fmt_rate(bps):
    for u, d in (("GB/s", 1e9), ("MB/s", 1e6), ("kB/s", 1e3)):
        if abs(bps) >= d:
            return "%.3f %s" % (bps / d, u)
    return "%.0f B/s" % bps


def fmt_bytes(n):
    for u, d in (("GiB", 1 << 30), ("MiB", 1 << 20), ("KiB", 1 << 10)):
        if abs(n) >= d:
            return "%.3f %s" % (n / d, u)
    return "%d B" % n


def fmt_time_s(s):
    if s >= 1:
        return "%.3f s" % s
    if s >= 1e-3:
        return "%.3f ms" % (s * 1e3)
    return "%.3f us" % (s * 1e6)


def hexn(v, width):
    """Fixed-width upper-case hex, as the protocol requires."""
    if v < 0 or v >= 1 << (4 * width):
        raise ValueError("value %d does not fit h%d" % (v, width))
    return "%0*X" % (width, v)


# ---------------------------------------------------------------------------
# Command formatting (spec 2.3) - all numbers fixed-width hex
# ---------------------------------------------------------------------------
def _chk(cond, msg):
    if not cond:
        raise ValueError(msg)


def cmd_ping(): return "PING"
def cmd_stat(): return "STAT?"
def cmd_cfg(): return "CFG?"
def cmd_res(): return "RES?"
def cmd_prof_q(): return "PROF?"
def cmd_ddr(): return "DDR?"
def cmd_smart_q(): return "SMART?"
def cmd_stop(): return "STOP"
def cmd_pfail(): return "PFAIL"
def cmd_abort(): return "ABORT"
def cmd_mon(): return "MON"
def cmd_dtest(): return "DTEST"
def cmd_prof_clr(): return "PROF CLR"
def cmd_format(): return "FORMAT YES"
def cmd_read_last(): return "READ LAST"
def cmd_read_end(): return "READ END"


def cmd_tel(ms):
    _chk(ms == 0 or ms >= 0x0A, "TEL: 0 or >= 10 ms")
    return "TEL " + hexn(ms, 4)


def cmd_smart(ms):
    _chk(ms == 0 or ms >= 0x64, "SMART: 0 or >= 100 ms")
    return "SMART " + hexn(ms, 4)


def cmd_mode(bp): return "MODE " + ("BP" if bp else "RT")
def cmd_rate(step): return "RATE " + hexn(step, 8)
def cmd_len(slots): return "LEN " + hexn(slots, 8)
def cmd_sid(sid): return "SID " + hexn(sid, 8)


def cmd_ddrsz(mib):
    _chk(1 <= mib <= 0x400, "DDRSZ: 1..1024 MiB")
    return "DDRSZ " + hexn(mib, 4)


def cmd_prof_add(step, dur):
    _chk(dur >= 1, "PROF ADD: dur >= 1 cycle")
    return "PROF ADD %s %s" % (hexn(step, 8), hexn(dur, 10))


def cmd_prof_set(i, step, dur):
    _chk(dur >= 1, "PROF SET: dur >= 1 cycle")
    return "PROF SET %s %s %s" % (hexn(i, 2), hexn(step, 8), hexn(dur, 10))


def cmd_prof_rep(n): return "PROF REP " + hexn(n, 4)
def cmd_prof_end(stop): return "PROF END " + ("STOP" if stop else "HOLD")
def cmd_rec(prof=False): return "REC PROF" if prof else "REC"


def cmd_read(slots):
    _chk(slots > 0, "READ: slots > 0")
    return "READ " + hexn(slots, 8)


def cmd_read_sid(sid): return "READ SID " + hexn(sid, 8)


def cmd_memtest(mib=None):
    return "MEMTEST" if mib is None else "MEMTEST " + hexn(mib, 4)


def check_line(line):
    if len(line) > MAX_LINE:
        raise ValueError("line > 64 chars")
    return line


# ---------------------------------------------------------------------------
# Reply / telemetry parsing (spec 2.3-2.6)
# ---------------------------------------------------------------------------
def _spec(s):
    """'v:2 t:8' -> [(key, width)]; width 0/absent = unspecified."""
    out = []
    for tok in s.split():
        k, _, w = tok.partition(":")
        out.append((k, int(w) if w else 0))
    return out


T_SPEC = _spec("v:2 t:8 st:1 md:1 cok:1 uok:1 cs:1 ce:2 cd:2 fl:2 lk:1 cal:1 stp:8 seg:2 "
               "rep:4 src:12 wr:12 drp:8 ddr:8 ddrm:8 bpm:10 rng:8 ssd:16 cmd:8 cer:8 lat:8 "
               "vb:12 ve:8 tk:4 cw:2 smo:4 sme:4 xt:3 xvi:3 xva:3 xvb:3")

SMART_FULL = _spec("cw:2 tk:4 sp:2 us:2 dur:16 duw:16 pc:8 poh:8 uns:8 me:8 wt:8 ct:8 "
                   "t1:4 ok:4 er:4 st:4 rtd:8")

# OK replies: name -> list of alternative ordered key lists
OK_SPEC = {
    "PING": [_spec("ver:2 bld:8 cal:1 uok:1")],
    "CFG": [_spec("md:1 bp:1 stp:8 len:8 sid:8 ddr:4 tel:4 sm:4 pn:2 pr:4 pe:1 rsid:8")],
    "TEL": [_spec("tel:4")],
    "SMART": [_spec("sm:4"), SMART_FULL],
    "MODE": [_spec("bp:1")],
    "RATE": [_spec("stp:8")],
    "LEN": [_spec("len:8")],
    "SID": [_spec("sid:8")],
    "DDRSZ": [_spec("ddr:4")],
    "PROF": [_spec("pn:2"), _spec("i:2"), _spec("pr:4"), _spec("pe:1"),
             _spec("pn:2 pr:4 pe:1")],
    "REC": [_spec("sid:8")],
    "STOP": [[]], "PFAIL": [[]], "ABORT": [[]],
    "READ": [[], _spec("rsid:8")],
    "MEMTEST": [[], _spec("mib:4")],
    "DDR": [_spec("cal:1 ddr:8 ddrm:8 mte:8 mta:8")],
    # READ / FORMAT / MON / MEMTEST / DTEST: reply fields not defined in the spec
}

EV_SPEC = {
    "B": _spec("ver:2 cal:1 uok:1"),
    "S": _spec("st:1 md:1 t:8"),
    "E": _spec("ce:2 cd:2 fl:2 st:1"),
    # widths of !RR fields other than sid/dur are not given in the spec (INTEGRATION_NOTES WP9)
    "RR": _spec("sid:8 wr drp pad trc ddrm ssd cmd cer lat dur:16"),
    "RP": _spec("src:1 inb:1 lat:1 lov:1 pnd:12 pdd:8 prg:8 pad:8 t1:16 t2:16 t3:16 t4:16 "
                "t5:16 t6:16 ok:1 rtd:8 ssd:16 lpos:10"),
    "RV": _spec("vb:12 ok:10 bad:8 crc:8 pe:8 ooo:8 prt:8 fbp:10 mvp:10 lsq:16 lts:16 lst:8 "
                "pad:8 fep:10 feb:2 exp:8 got:8 end:1 ovf:1 lfl:2 dur:16"),   # lfl: flags of the max-valid block (bit 1 = LAST)
    "RF": _spec("ms:8 ok:1"),
    "RM": _spec("byt:8 err:8 adr:8 dur:16"),
    "RD": _spec("wr:12 drp:8 he:8 dur:16"),
}

P_SPEC = _spec("i:2 stp:8 dur:10")
_HEX = re.compile(r"[0-9A-Fa-f]+")


class Msg:
    """Parsed protocol line. f: key -> int (hex fields); s: key -> raw string."""
    def __init__(self, kind, name, f, s, order, text):
        self.kind, self.name, self.f, self.s, self.order, self.text = kind, name, f, s, order, text

    def __getitem__(self, k):
        return self.f[k]

    def get(self, k, d=None):
        return self.f.get(k, d)

    @property
    def is_reply(self):
        return self.kind in ("OK", "ERR")

    @property
    def is_async(self):
        return self.kind in ("T", "EV")

    def __repr__(self):
        return "Msg(%s %s %s)" % (self.kind, self.name, self.f)


def _fields(tokens, line):
    f, s, order = {}, {}, []
    for tk in tokens:
        k, eq, v = tk.partition("=")
        if not eq or not k or not v:
            raise ParseError("bad field %r in %r" % (tk, line))
        if k in s:
            raise ParseError("duplicate key %r in %r" % (k, line))
        s[k] = v
        order.append(k)
        if _HEX.fullmatch(v):
            f[k] = int(v, 16)
    return f, s, order


def _check_spec(spec, f, s, order, line, ordered=True, allow_extra=True):
    """Verify widths (where known), presence and (optionally) order of spec keys."""
    for k, w in spec:
        if k not in s:
            raise ParseError("missing key %r in %r" % (k, line))
        if k not in f:
            raise ParseError("key %r is not hex in %r" % (k, line))
        if w and len(s[k]) != w:
            raise ParseError("key %r: width %d, expected h%d in %r" % (k, len(s[k]), w, line))
    if ordered:
        want = [k for k, _ in spec]
        got = [k for k in order if k in want]
        if got != want:
            raise ParseError("key order %s != %s in %r" % (got, want, line))
    if not allow_extra and len(order) != len(spec):
        raise ParseError("unexpected extra keys in %r" % line)


def parse_line(line, strict=True):
    """Parse one device line.  Returns Msg, or None for an empty line.
    strict: verify field widths / order / presence against the spec."""
    text = line.strip()
    if not text:
        return None
    tok = text.split()
    head = tok[0]
    if head == "$T":
        f, s, order = _fields(tok[1:], text)
        if strict:
            _check_spec(T_SPEC, f, s, order, text, allow_extra=False)
            if f["v"] != 1:
                raise ParseError("unsupported $T version %02X" % f["v"])
        return Msg("T", "T", f, s, order, text)
    if head.startswith("!"):
        name = head[1:]
        f, s, order = _fields(tok[1:], text)
        if strict and name in EV_SPEC:
            _check_spec(EV_SPEC[name], f, s, order, text)
        return Msg("EV", name, f, s, order, text)
    if head == "OK":
        name = ""
        rest = tok[1:]
        if rest and "=" not in rest[0]:
            name, rest = rest[0], rest[1:]
        f, s, order = _fields(rest, text)
        if strict and name in OK_SPEC:
            alts = OK_SPEC[name]
            for alt in alts:
                if [k for k, _ in alt] == order:
                    _check_spec(alt, f, s, order, text, allow_extra=False)
                    break
            else:
                raise ParseError("OK %s: fields %s match no known form" % (name, order))
        return Msg("OK", name, f, s, order, text)
    if head == "ERR":
        if len(tok) != 2 or not _HEX.fullmatch(tok[1]) or len(tok[1]) != 2:
            raise ParseError("bad ERR line %r" % text)
        return Msg("ERR", "", {"code": int(tok[1], 16)}, {"code": tok[1]}, ["code"], text)
    if head == "P":
        f, s, order = _fields(tok[1:], text)
        if strict:
            _check_spec(P_SPEC, f, s, order, text, allow_extra=False)
        return Msg("P", "P", f, s, order, text)
    raise ParseError("unrecognised line %r" % text)


def format_fields(spec, values):
    """Build 'k=HEX ...' string in spec order (device side / tests)."""
    out = []
    for k, w in spec:
        out.append("%s=%s" % (k, hexn(values.get(k, 0), w) if w else "%X" % values.get(k, 0)))
    return " ".join(out)


def format_telemetry(values):
    return "$T " + format_fields(T_SPEC, values)


# ---------------------------------------------------------------------------
# Derived (human) values
# ---------------------------------------------------------------------------
def telemetry_derived(m):
    """Convert a parsed $T into engineering units (dict)."""
    f = m.f
    d = {
        "t_s": f["t"] / 1000.0,
        "state": STATES.get(f["st"], "?%X" % f["st"]),
        "mode": MODES.get(f["md"], "?%X" % f["md"]),
        "core_state": CORE_STATES.get(f["cs"], "?%X" % f["cs"]),
        "link_up": f["lk"] & 1, "gen2": f["lk"] >> 1 & 1, "lanes": 1 << (f["lk"] >> 2 & 3),   # lk[3:2] = log2(lanes) (D16)
        "rate_Bps": step_to_rate(f["stp"]),
        "src_bytes": f["src"] * 4096, "wr_bytes": f["wr"] * 4096,
        "ssd_bytes": f["ssd"], "ddr_bytes": f["ddr"], "ddr_max_bytes": f["ddrm"],
        "range_bytes": f["rng"] * 16, "lat_us": f["lat"] * 8e-3,
        "smart_temp_C": (f["tk"] - 273.15) if f["tk"] else None,
        "xadc_temp_C": xadc_temp_c(f["xt"]),
        "vccint_V": xadc_volt(f["xvi"]), "vccaux_V": xadc_volt(f["xva"]),
        "vccbram_V": xadc_volt(f["xvb"]),
    }
    return d


def result_derived(m):
    """Convert !R* event times (5 ns ticks) to seconds."""
    d = {}
    for k, v in m.f.items():
        if k == "dur" or (m.name == "RP" and k in ("t1", "t2", "t3", "t4", "t5", "t6")):
            d[k + "_s"] = v * LINK_CLK_NS * 1e-9
    return d


def smart_derived(m):
    f = m.f
    d = {"temp_C": f["tk"] - 273.15 if f["tk"] else None}
    if "dur" in f:
        d["data_read_bytes"] = f["dur"] * 512000      # NVMe data unit = 1000 x 512 B
        d["data_written_bytes"] = f["duw"] * 512000
    return d


HEX_KEYS = ("bld", "sid", "rsid")   # identifiers shown in hex, not decimal


def pretty(m):
    """Human-readable one-line form with decimal numbers and units."""
    if m.kind == "T":
        d = telemetry_derived(m)
        return ("T=%.3fs %s/%s core=%s rate=%s ssd=%s ddr=%s (max %s) drp=%d verified=%s "
                "err_clk=%d Tj=%.1fC VCCINT=%.3fV%s" % (
                    d["t_s"], d["state"], d["mode"], d["core_state"], fmt_rate(d["rate_Bps"]),
                    fmt_bytes(d["ssd_bytes"]), fmt_bytes(d["ddr_bytes"]),
                    fmt_bytes(d["ddr_max_bytes"]), m["drp"], fmt_bytes(m["vb"]), m["ve"],
                    d["xadc_temp_C"], d["vccint_V"],
                    "" if d["smart_temp_C"] is None else " SSD=%.0fC" % d["smart_temp_C"]))
    if m.kind == "ERR":
        return "ERR %02X: %s" % (m["code"], ERR_TEXT.get(m["code"], "?"))
    items = [("%s=%08X" if k in HEX_KEYS else "%s=%d") % (k, v) for k, v in m.f.items()]
    extra = {}
    head = {"OK": "OK " + m.name, "EV": "!" + m.name, "P": "P"}.get(m.kind, m.kind)
    if m.kind == "EV":
        extra = result_derived(m)
    elif m.kind == "OK" and m.name == "SMART" and "tk" in m.f:
        extra = smart_derived(m)
    elif m.kind == "OK" and "stp" in m.f:
        extra = {"rate": fmt_rate(step_to_rate(m["stp"]))}
    elif m.kind == "P":
        extra = {"rate": fmt_rate(step_to_rate(m["stp"])),
                 "dur": fmt_time_s(cycles_to_us(m["dur"]) * 1e-6)}
    items += ["%s=%s" % (k, ("%.6g" % v) if isinstance(v, float) else v)
              for k, v in extra.items()]
    return (head + " " + " ".join(items)).rstrip()


# ---------------------------------------------------------------------------
# FPGA power estimate (spec 8.4) - offline table + XADC temperature + mode
# ---------------------------------------------------------------------------
POWER_COLS = ["mode", "rate_Bps", "tj_C", "p_static_W", "p_dyn_W", "p_vccint_W",
              "p_vccaux_W", "p_vcco_W", "p_mgt_W"]
P_NOTE = "VECTORLESS_ESTIMATE_+-30%"
# md code -> power-table mode name.  FORMAT ~ MON (no data), MEMTEST/DTEST use DDR (REC row)
MD_TO_POWER_MODE = {0: "IDLE", 1: "REC", 2: "REC", 3: "READ", 4: "MON", 5: "MON",
                    6: "REC", 7: "REC"}


def _lin(xs, ys, x):
    """Piecewise-linear interpolation, clamped at the ends. xs ascending."""
    if len(xs) == 1 or x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    i = bisect.bisect_right(xs, x)
    x0, x1 = xs[i - 1], xs[i]
    return ys[i - 1] + (ys[i] - ys[i - 1]) * (x - x0) / (x1 - x0)


class PowerTable:
    def __init__(self, rows):
        self.rows = rows            # list of dicts with float values, 'mode' str
        self.modes = sorted({r["mode"] for r in rows})
        if not rows:
            raise ValueError("empty power table")

    @classmethod
    def load(cls, path):
        with open(path, newline="") as f:
            rd = csv.DictReader(f)
            miss = [c for c in POWER_COLS if c not in (rd.fieldnames or [])]
            if miss:
                raise ValueError("power table %s: missing columns %s" % (path, miss))
            rows = []
            for r in rd:
                rr = {"mode": r["mode"].strip().upper()}
                for c in POWER_COLS[1:]:
                    rr[c] = float(r[c])
                rows.append(rr)
        return cls(rows)

    def total(self, r):
        return r["p_static_W"] + r["p_dyn_W"]

    def estimate(self, mode, rate_bps, tj_c, col=None):
        """Bilinear (rate, Tj) interpolation inside one mode; clamped to the table."""
        mode = mode.upper()
        rows = [r for r in self.rows if r["mode"] == mode]
        if not rows:
            raise KeyError("no rows for mode %s (have %s)" % (mode, self.modes))
        val = (lambda r: r[col]) if col else self.total
        tjs = sorted({r["tj_C"] for r in rows})
        per_tj = []
        for tj in tjs:
            rs = sorted((r for r in rows if r["tj_C"] == tj), key=lambda r: r["rate_Bps"])
            per_tj.append(_lin([r["rate_Bps"] for r in rs], [val(r) for r in rs], rate_bps))
        return _lin(tjs, per_tj, tj_c)

    def estimate_md(self, md, rate_bps, tj_c):
        return self.estimate(MD_TO_POWER_MODE.get(md, "IDLE"), rate_bps, tj_c)


# ---------------------------------------------------------------------------
# CSV telemetry log
# ---------------------------------------------------------------------------
CSV_DERIVED = ["t_s", "state", "mode", "core_state", "rate_Bps", "ssd_rate_Bps", "ddr_bytes",
               "xadc_temp_C", "vccint_V", "vccaux_V", "vccbram_V", "smart_temp_C",
               "p_fpga_est_W", "p_note", "board_w", "host_time"]
CSV_COLUMNS = [k for k, _ in T_SPEC] + CSV_DERIVED


class TelemetryCsv:
    def __init__(self, path_or_file, power=None, board_w=None):
        self._own = isinstance(path_or_file, str)
        self.f = open(path_or_file, "w", newline="") if self._own else path_or_file
        self.w = csv.DictWriter(self.f, fieldnames=CSV_COLUMNS)
        self.w.writeheader()
        self.power, self.board_w = power, board_w
        self._prev = None
        self.rows = 0

    def add(self, m, host_time=None):
        d = telemetry_derived(m)
        row = dict(m.f)
        ssd_rate = ""
        if self._prev is not None and m["t"] > self._prev[0]:
            ssd_rate = "%.1f" % ((m["ssd"] - self._prev[1]) * 1000.0 / (m["t"] - self._prev[0]))
        self._prev = (m["t"], m["ssd"])
        row.update(t_s="%.3f" % d["t_s"], state=d["state"], mode=d["mode"],
                   core_state=d["core_state"], rate_Bps="%.1f" % d["rate_Bps"],
                   ssd_rate_Bps=ssd_rate, ddr_bytes=d["ddr_bytes"],
                   xadc_temp_C="%.2f" % d["xadc_temp_C"], vccint_V="%.4f" % d["vccint_V"],
                   vccaux_V="%.4f" % d["vccaux_V"], vccbram_V="%.4f" % d["vccbram_V"],
                   smart_temp_C="" if d["smart_temp_C"] is None else "%.2f" % d["smart_temp_C"],
                   board_w="" if self.board_w is None else "%g" % self.board_w,
                   host_time="%.3f" % (time.time() if host_time is None else host_time))
        if self.power is not None:
            p = self.power.estimate_md(m["md"], d["rate_Bps"], d["xadc_temp_C"])
            row["p_fpga_est_W"] = "%.3f" % p
            row["p_note"] = P_NOTE
        else:
            row["p_fpga_est_W"] = ""
            row["p_note"] = "NO_POWER_TABLE"
        self.w.writerow(row)
        self.rows += 1

    def close(self):
        if self._own:
            self.f.close()


# ---------------------------------------------------------------------------
# Profile file ("280MB/s 30s" per line, '#' comments)
# ---------------------------------------------------------------------------
def parse_profile_text(text):
    """-> list of (step, dur_cycles).  Line: <rate> <duration>; rate 0 = pause."""
    segs = []
    for n, ln in enumerate(text.splitlines(), 1):
        ln = ln.split("#")[0].strip()
        if not ln:
            continue
        p = ln.split()
        if len(p) != 2:
            raise ValueError("profile line %d: expected '<rate> <duration>'" % n)
        step = rate_to_step(parse_rate(p[0]))
        dur = us_to_cycles(parse_duration(p[1]) * 1e6)
        if dur < 1 or dur > DUR_MAX:
            raise ValueError("profile line %d: duration out of range" % n)
        segs.append((step, dur))
    if not segs:
        raise ValueError("empty profile")
    if len(segs) > MAX_SEGMENTS:
        raise ValueError("profile has %d segments, max %d" % (len(segs), MAX_SEGMENTS))
    return segs


# ---------------------------------------------------------------------------
# Transports
# ---------------------------------------------------------------------------
class SerialTransport:
    def __init__(self, port, baud=BAUD):
        try:
            import serial           # lazy: offline parts must work without pyserial
        except ImportError:
            raise RecError("pyserial is not installed: pip install pyserial (or use --fake)")
        self.ser = serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1, timeout=0.05)
        self._buf = bytearray()

    def write(self, data):
        self.ser.write(data)

    def readline(self, timeout):
        end = time.monotonic() + timeout
        while True:
            for i, c in enumerate(self._buf):
                if c in (10, 13):
                    line = bytes(self._buf[:i]).decode("ascii", "replace")
                    del self._buf[:i + 1]
                    if line.strip():
                        return line
                    break               # empty line (CRLF second char): rescan
            else:
                if time.monotonic() >= end:
                    return None
                self._buf += self.ser.read(256)

    def close(self):
        self.ser.close()


class FakeDevice:
    """Behavioural model of the recorder UART interface (not of the hardware).
    Accepts spec 2.3 commands, returns spec-format replies, runs a virtual clock."""

    def __init__(self, cal=1):
        self.cal, self.ver, self.bld = cal, 2, 0x20261201
        self.t_ms = 1
        self.md = 0
        self.st = 0
        self.bp = 0
        self.stp = 0
        self.len = 0
        self.sid = 0x692D3A00
        self.sid_set = False
        self.ddr = 0x400
        self.tel, self.sm = 0x3E8, 0x3E8
        self.prof, self.pr, self.pe = [], 1, 1
        self.src = 0
        self.ssd = 0
        self.last_res = None
        self.wr_blocks = 0
        self.out = []
        self._run_start = 0
        self.temp = 45.0

    # -- helpers
    def _t(self, **kv):
        v = dict(v=1, t=self.t_ms, st=self.st, md=self.md, cok=1, uok=1,
                 cs=5 if self.st == 2 else 0, ce=0, cd=0, fl=0, lk=0x9, cal=self.cal,
                 stp=self.stp, seg=0, rep=0, src=self.src, wr=self.src, drp=0,
                 ddr=0, ddrm=0, bpm=0, rng=0, ssd=self.ssd, cmd=self.ssd // 131072,
                 cer=0, lat=0x1000, vb=0, ve=0, tk=0x136, cw=0, smo=self.t_ms // max(1, self.sm),
                 sme=0, xt=temp_c_to_xadc(self.temp + (2 if self.st == 2 else 0)),
                 xvi=volt_to_xadc(1.0), xva=volt_to_xadc(1.8), xvb=volt_to_xadc(1.0))
        v.update(kv)
        return format_telemetry(v)

    def _ev_s(self):
        return "!S st=%X md=%X t=%s" % (self.st, self.md, hexn(self.t_ms, 8))

    def _set_state(self, st, md):
        self.st, self.md = st, md
        self.out.append(self._ev_s())

    def _finish(self):
        self.prof_active = False
        wr = self.src
        self.last_res = ("!RR sid=%s wr=%s drp=00000000 pad=00000000 trc=00000001 "
                         "ddrm=00000000 ssd=%s cmd=%s cer=00000000 lat=00001000 dur=%s" % (
                             hexn(self.sid, 8), hexn(wr, 12), hexn(self.ssd, 16),
                             hexn(self.ssd // 131072, 8),
                             hexn((self.t_ms - self._run_start) * 200000, 16)))
        self.wr_blocks = wr
        self._set_state(0, 0)
        self.out.append(self.last_res)

    def advance(self, ms):
        """Advance virtual time; emit telemetry/auto-stop."""
        end = self.t_ms + ms
        while self.t_ms < end:
            step = min(end - self.t_ms, 1)
            self.t_ms += step
            if self.st == 2 and self.md == 1:
                dt = step / 1000.0
                self._acc += step_to_rate(self.stp) * dt
                blocks = int(self._acc // 4096)
                self._acc -= blocks * 4096
                self.src += blocks
                self.ssd = (self.src // 32) * 131072
                limit = self.len * 32
                if self.len and self.src >= limit:
                    self.src = limit
                    self.ssd = limit * 4096
                    self._finish()
                    continue
            if self.tel and self.t_ms % self.tel == 0:
                self.out.append(self._t())

    # -- command handling
    def _ok(self, s=""):
        self.out.append(("OK " + s).strip())

    def _err(self, code):
        self.out.append("ERR %02X" % code)

    def write_line(self, line):
        if len(line) > MAX_LINE:
            return self._err(0x0A)
        tok = line.split()
        if not tok:
            return
        c = tok[0].upper()
        a = tok[1:]
        idle = self.st == 0

        def num(i, lo=0, hi=None, maxdig=16):
            if len(a) <= i or not _HEX.fullmatch(a[i]) or len(a[i]) > maxdig:
                raise ValueError(1)
            v = int(a[i], 16)
            if v < lo or (hi is not None and v > hi):
                raise ValueError(3)
            return v

        try:
            self._dispatch(c, a, idle, num)
        except ValueError as e:
            self._err(e.args[0] if e.args and isinstance(e.args[0], int) else 1)

    def _dispatch(self, c, a, idle, num):
        if c == "PING":
            return self._ok("PING ver=%02X bld=%08X cal=%X uok=1" % (self.ver, self.bld, self.cal))
        if c == "STAT?":
            self.out.append(self._t())
            return self._ok()
        if c == "CFG?":
            return self._ok("CFG md=%X bp=%X stp=%s len=%s sid=%s ddr=%s tel=%s sm=%s pn=%s "
                            "pr=%s pe=%X rsid=%s" % (
                                self.md, self.bp, hexn(self.stp, 8), hexn(self.len, 8),
                                hexn(self.sid, 8), hexn(self.ddr, 4), hexn(self.tel, 4),
                                hexn(self.sm, 4), hexn(len(self.prof), 2), hexn(self.pr, 4),
                                self.pe, hexn(self.sid, 8)))
        if c == "RES?":
            if self.last_res:
                self.out.append(self.last_res)
            return self._ok()
        if c == "TEL":
            v = num(0, 0, 0xFFFF, 4)
            if 0 < v < 10:
                raise ValueError(3)
            self.tel = v
            return self._ok("TEL tel=%s" % hexn(v, 4))
        if c == "SMART":
            v = num(0, 0, 0xFFFF, 4)
            if 0 < v < 100:
                raise ValueError(3)
            self.sm = v
            return self._ok("SMART sm=%s" % hexn(v, 4))
        if c == "SMART?":
            return self._ok("SMART cw=00 tk=0136 sp=64 us=05 dur=%s duw=%s pc=00000010 "
                            "poh=00000100 uns=00000002 me=00000000 wt=00000130 ct=00000000 "
                            "t1=0000 ok=%s er=0000 st=0000 rtd=000186A0" % (
                                hexn(1000, 16), hexn(self.ssd // 512000, 16),
                                hexn(self.t_ms // max(1, self.sm), 4)))
        if c == "MODE":
            if not idle:
                raise ValueError(4)
            if len(a) != 1 or a[0].upper() not in ("RT", "BP"):
                raise ValueError(1)
            self.bp = int(a[0].upper() == "BP")
            return self._ok("MODE bp=%X" % self.bp)
        if c == "RATE":
            v = num(0, 0, STEP_MAX, 8)
            if not (idle or (self.st in (2, 6) and not self.prof_active)):
                raise ValueError(4)
            self.stp = v
            return self._ok("RATE stp=%s" % hexn(v, 8))
        if c in ("LEN", "SID", "DDRSZ", "PROF", "READ", "FORMAT", "MON", "MEMTEST", "DTEST",
                 "REC") and not idle and c != "PROF":
            raise ValueError(4)
        if c == "LEN":
            self.len = num(0, 0, STEP_MAX, 8)
            return self._ok("LEN len=%s" % hexn(self.len, 8))
        if c == "SID":
            self.sid, self.sid_set = num(0, 0, STEP_MAX, 8), True
            return self._ok("SID sid=%s" % hexn(self.sid, 8))
        if c == "DDRSZ":
            self.ddr = num(0, 1, 0x400, 4)
            return self._ok("DDRSZ ddr=%s" % hexn(self.ddr, 4))
        if c == "PROF?":
            for i, (s, d) in enumerate(self.prof):
                self.out.append("P i=%s stp=%s dur=%s" % (hexn(i, 2), hexn(s, 8), hexn(d, 10)))
            return self._ok("PROF pn=%s pr=%s pe=%X" % (hexn(len(self.prof), 2),
                                                        hexn(self.pr, 4), self.pe))
        if c == "PROF":
            if not idle:
                raise ValueError(4)
            sub = a[0].upper() if a else ""
            del a[:1]
            if sub == "CLR":
                self.prof = []
                return self._ok("PROF pn=00")
            if sub == "ADD":
                s, d = num(0, 0, STEP_MAX, 8), num(1, 1, DUR_MAX, 10)
                if len(self.prof) >= MAX_SEGMENTS:
                    raise ValueError(9)
                self.prof.append((s, d))
                return self._ok("PROF i=%s" % hexn(len(self.prof) - 1, 2))
            if sub == "SET":
                i, s, d = num(0, 0, 0xFF, 2), num(1, 0, STEP_MAX, 8), num(2, 1, DUR_MAX, 10)
                if i >= len(self.prof):
                    raise ValueError(3)
                self.prof[i] = (s, d)
                return self._ok("PROF i=%s" % hexn(i, 2))
            if sub == "REP":
                self.pr = num(0, 0, 0xFFFF, 4)
                return self._ok("PROF pr=%s" % hexn(self.pr, 4))
            if sub == "END":
                w = a[0].upper() if a else ""
                if w not in ("HOLD", "STOP"):
                    raise ValueError(1)
                self.pe = int(w == "STOP")
                return self._ok("PROF pe=%X" % self.pe)
            raise ValueError(1)
        if c == "REC":
            if not self.cal:
                raise ValueError(5)
            usep = bool(a) and a[0].upper() == "PROF"
            if usep and not self.prof:
                raise ValueError(8)
            if usep:
                self.stp = self.prof[0][0]
            self.prof_active = usep
            if not self.sid_set and self.md == 0 and self.wr_blocks:
                self.sid = (self.sid + 1) & 0xFFFFFFFF
            self.src = self.ssd = 0
            self._acc = 0.0
            self._run_start = self.t_ms
            self._ok("REC sid=%s" % hexn(self.sid, 8))
            self._set_state(1, 1)
            self._set_state(2, 1)
            return
        if c in ("STOP", "PFAIL"):
            if self.st not in (1, 2, 6) and not (c == "PFAIL" and self.st in (3, 4)):
                raise ValueError(4)
            self._ok(c)
            if self.md in (1, 2):
                self.md = 2 if c == "PFAIL" else self.md
                self._finish()
            else:
                self._set_state(0, 0)
            return
        if c == "READ":
            if not self.cal:
                raise ValueError(5)
            sub = a[0].upper() if a else ""
            if sub == "LAST":
                if not self.wr_blocks:
                    raise ValueError(0xB)
            elif sub == "END":
                pass
            elif sub == "SID":
                v = num(1, 0, STEP_MAX, 8)
                return self._ok("READ rsid=%s" % hexn(v, 8))
            else:
                num(0, 1, STEP_MAX, 8)
            self._ok("READ")
            self.out.append("!RV vb=%s ok=%s bad=00000000 crc=00000000 pe=00000000 "
                            "ooo=00000000 prt=00000000 fbp=%s mvp=%s lsq=%s lts=%s lst=00000000 "
                            "pad=00000000 fep=%s feb=00 exp=00000000 got=00000000 end=1 ovf=0 "
                            "lfl=02 dur=%s" % (hexn(self.wr_blocks * 4096, 12),
                                        hexn(self.wr_blocks, 10), hexn(self.wr_blocks, 10),
                                        hexn(max(self.wr_blocks - 1, 0), 10) if self.wr_blocks
                                        else "F" * 10, hexn(self.wr_blocks, 16), hexn(0, 16),
                                        "F" * 10, hexn(1000000, 16)))
            return
        if c == "FORMAT":
            if not a or a[0].upper() != "YES":
                raise ValueError(1)
            self._ok("FORMAT")
            self.out.append("!RF ms=%s ok=1" % hexn(1200, 8))
            self.wr_blocks = 0
            return
        if c in ("MON", "DTEST", "MEMTEST"):
            if c == "MEMTEST" and a:
                num(0, 1, 0x400, 4)
            if c != "MON" and not self.cal:
                raise ValueError(5)
            if c == "MEMTEST":
                mib = int(a[0], 16) if a else self.ddr
                self._ok("MEMTEST mib=%s" % hexn(mib, 4))
            else:
                self._ok(c)
            if c == "MEMTEST":
                self.out.append("!RM byt=%s err=00000000 adr=00000000 dur=%s" % (
                    hexn(self.ddr << 20 & 0xFFFFFFFF, 8), hexn(10 ** 8, 16)))
            elif c == "MON":
                self._set_state(1, 5)
                self._set_state(2, 5)
            else:
                self._set_state(6, 6)
            return
        if c == "DDR?":
            return self._ok("DDR cal=%X ddr=00000000 ddrm=00000000 mte=00000000 mta=00000000"
                            % self.cal)
        if c == "ABORT":
            self._ok("ABORT")
            self.st = self.md = 0
            return
        raise ValueError(2)

    prof_active = False
    _acc = 0.0

    # -- transport side
    def write(self, data):
        for ln in data.decode("ascii", "replace").replace("\r", "\n").split("\n"):
            if ln.strip():
                self.write_line(ln)

    def readline(self, timeout):
        if not self.out:
            self.advance(max(1, int(min(timeout, 1.0) * 1000)))
        return self.out.pop(0) if self.out else None

    def close(self):
        pass


# ---------------------------------------------------------------------------
# Link (command/reply, async dispatch)
# ---------------------------------------------------------------------------
class Reply:
    def __init__(self, line, pre, msg):
        self.line, self.pre, self.msg = line, pre, msg

    @property
    def ok(self):
        return self.msg.kind == "OK"


class Link:
    def __init__(self, transport, eol="\n", timeout=2.0, strict=True, on_async=None):
        self.tp, self.eol, self.timeout, self.strict = transport, eol, timeout, strict
        self.on_async = on_async
        self.async_log = []
        self.stale = []             # OK/ERR lines that belonged to no pending command
        self._primed = False
        self._need_sync = False

    def _read_msg(self, timeout):
        ln = self.tp.readline(timeout)
        if ln is None:
            return None
        return parse_line(ln, self.strict)

    def _async(self, m):
        self.async_log.append(m)
        if self.on_async:
            self.on_async(m)

    ERR_GRACE = 0.05        # s: how long an ERR may wait for a later OK (see _exchange)

    @staticmethod
    def _expect_name(line):
        """OK keyword the reply to `line` must carry (OK_SPEC key); None = not checked."""
        tok = line.split()[0].upper().rstrip("?") if line.split() else ""
        return None if tok in ("STAT", "RES", "") else tok

    def _prime(self):
        """Once per link: a bare EOL ends any half-received line in the device parser."""
        if not self._primed:
            self._primed = True
            self.tp.write(self.eol.encode("ascii"))

    def _drain(self):
        """Discard input that arrived while no command was pending (late/stray OK, ERR)."""
        while True:
            try:
                m = self._read_msg(0.01)
            except ParseError:
                continue
            if m is None:
                return
            if m.kind in ("OK", "ERR"):
                self.stale.append(m)
            else:
                self._async(m)

    def _exchange(self, line, timeout, name_ok):
        """Send `line`; return (reply_msg, pre).  Replies are in command order (spec 2.1):
        an OK whose keyword does not match belongs to an earlier command -> stale; an ERR
        counts as the reply only if no matching OK follows it (ERR 0C/0D can be stray)."""
        self.tp.write((line + self.eol).encode("ascii"))
        wait_end = time.monotonic() + timeout
        pre, err = [], None
        while True:
            if err is not None:
                m = self._read_msg(self.ERR_GRACE)
                if m is None:
                    return err, pre
            else:
                left = wait_end - time.monotonic()
                if left <= 0:
                    raise TimeoutError("no reply to %r" % line)
                m = self._read_msg(left)
                if m is None:
                    continue
            if m.kind == "OK":
                if name_ok(m):
                    if err is not None:
                        self.stale.append(err)
                    return m, pre
                self.stale.append(m)
            elif m.kind == "ERR":
                if err is not None:
                    self.stale.append(err)
                err = m
            else:
                pre.append(m)
                self._async(m)

    def _resync(self):
        """After a timeout: PING until its own OK PING arrives; the device is then idle."""
        self._drain()
        try:
            self._exchange(cmd_ping(), self.timeout, lambda m: m.name == "PING")
        except TimeoutError:
            raise TimeoutError("link not resynchronised: no OK PING")
        self._drain()
        self._need_sync = False

    def command(self, line, raise_err=True, timeout=None):
        """Send one command; return Reply (pre = intermediate P/$T/! lines kept in order).
        Async lines ($T, !...) go to on_async; P rows and for STAT?/RES? the $T/!R*
        line preceding OK are returned in .pre.  Stale input is drained first; after a
        TimeoutError the next call resyncs with PING (review fix H)."""
        line = check_line(line)
        self._prime()
        if self._need_sync:
            self._resync()
        self._drain()
        want = self._expect_name(line)
        try:
            m, pre = self._exchange(line, timeout or self.timeout,
                                    lambda r: not (want and r.name and r.name != want))
        except TimeoutError:
            self._need_sync = True
            raise
        if m.kind == "ERR" and raise_err:
            raise DeviceError(m["code"], line)
        return Reply(line, pre, m)

    def poll(self, timeout=0.1):
        m = self._read_msg(timeout)
        if m is not None:
            self._async(m)
        return m

    # high level -------------------------------------------------------
    def ping(self):
        t0 = time.monotonic()
        r = self.command(cmd_ping())
        return r.msg, time.monotonic() - t0

    def upload_profile(self, segs, rep=1, end_stop=True):
        self.command(cmd_prof_clr())
        for s, d in segs:
            self.command(cmd_prof_add(s, d))
        self.command(cmd_prof_rep(rep))
        self.command(cmd_prof_end(end_stop))
        r = self.command(cmd_prof_q())
        rows = [(m["stp"], m["dur"]) for m in r.pre if m.kind == "P"]
        if rows != list(segs):
            raise RecError("profile read-back mismatch: %s != %s" % (rows, segs))
        return rows

    def log_telemetry(self, seconds, csv_out, wall_limit=None):
        """Poll $T lines for `seconds` of device time (field t); log them. Returns count."""
        t_first, n = None, 0
        wall_end = time.monotonic() + (wall_limit if wall_limit else seconds * 2 + 10)
        while time.monotonic() < wall_end:
            m = self.poll(0.2)
            if m is not None and m.kind == "T":
                csv_out.add(m)
                n += 1
                t_first = m["t"] if t_first is None else t_first
                if m["t"] - t_first >= seconds * 1000:
                    break
        return n


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _print_msg(m, raw=False):
    print(m.text if raw else pretty(m))


def make_link(a, on_async=None):
    tp = FakeDevice() if a.fake else SerialTransport(a.port, a.baud)
    eol = {"lf": "\n", "cr": "\r", "crlf": "\r\n"}[a.eol]
    return Link(tp, eol=eol, on_async=on_async)


def run_selftest():
    import unittest
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, here)
    suite = unittest.defaultTestLoader.discover(os.path.join(here, "tests"),
                                                pattern="test_*.py", top_level_dir=here)
    res = unittest.TextTestRunner(verbosity=1).run(suite)
    return 0 if res.wasSuccessful() else 1


def build_parser():
    ap = argparse.ArgumentParser(description="Recorder PC tool (see module docstring)")
    ap.add_argument("--selftest", action="store_true", help="run offline self-tests")
    ap.add_argument("--port", help="serial port (COMx, /dev/ttyUSBx)")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--fake", action="store_true", help="loopback to built-in fake device")
    ap.add_argument("--eol", choices=("lf", "cr", "crlf"), default="lf")
    ap.add_argument("--raw", action="store_true", help="print raw device lines")
    ap.add_argument("--log", metavar="CSV", help="write $T telemetry to CSV")
    ap.add_argument("--seconds", type=float, default=0, help="keep logging telemetry for N s")
    ap.add_argument("--power-table", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "fpga", "power", "power_table.csv"))
    ap.add_argument("--board-w", type=float, help="board power from an external meter, W")
    sub = ap.add_subparsers(dest="cmd")
    p = sub.add_parser("ping"); p.add_argument("--count", type=int, default=1)
    for n in ("stat", "cfg", "res", "ddr", "stop", "pfail", "abort", "mon", "dtest",
              "smart-q", "prof-q", "format"):
        sub.add_parser(n)
    p = sub.add_parser("tel"); p.add_argument("ms", type=lambda x: int(x, 0))
    p = sub.add_parser("smart"); p.add_argument("ms", type=lambda x: int(x, 0))
    p = sub.add_parser("mode"); p.add_argument("which", choices=("RT", "BP", "rt", "bp"))
    p = sub.add_parser("rate"); p.add_argument("rate", help="e.g. 280MB/s")
    p = sub.add_parser("sid"); p.add_argument("sid", type=lambda x: int(x, 16))
    p = sub.add_parser("ddrsz"); p.add_argument("size", help="e.g. 32MiB")
    p = sub.add_parser("rec")
    p.add_argument("--rate"); p.add_argument("--len", dest="length", help="e.g. 4MiB")
    p.add_argument("--sid", type=lambda x: int(x, 16)); p.add_argument("--profile", action="store_true")
    p = sub.add_parser("prof")
    p.add_argument("--file", required=True); p.add_argument("--rep", type=int, default=1)
    p.add_argument("--end", choices=("stop", "hold"), default="stop")
    p = sub.add_parser("read"); p.add_argument("what", help="LAST | END | <size e.g. 64MiB>")
    p.add_argument("--sid", type=lambda x: int(x, 16))
    p = sub.add_parser("memtest"); p.add_argument("size", nargs="?")
    p = sub.add_parser("raw"); p.add_argument("line")
    p = sub.add_parser("decode"); p.add_argument("file")
    p = sub.add_parser("convert"); p.add_argument("what", choices=("rate", "step", "us", "cycles"))
    p.add_argument("value")
    return ap


def main(argv=None):
    ap = build_parser()
    a = ap.parse_args(argv)
    if a.selftest:
        return run_selftest()
    if not a.cmd:
        ap.print_help()
        return 2
    # offline commands
    if a.cmd == "convert":
        if a.what == "rate":
            st = rate_to_step(parse_rate(a.value))
            print("step=%08X actual=%s" % (st, fmt_rate(step_to_rate(st))))
        elif a.what == "step":
            print(fmt_rate(step_to_rate(int(a.value, 16))))
        elif a.what == "us":
            print("cycles=%d (h10 %s)" % (us_to_cycles(float(a.value)),
                                          hexn(us_to_cycles(float(a.value)), 10)))
        else:
            print("%.3f us" % cycles_to_us(int(a.value, 16)))
        return 0
    if a.cmd == "decode":
        import rec_decode
        return rec_decode.main(["decode", a.file])
    if not a.fake and not a.port:
        ap.error("--port or --fake required")
    power = None
    if os.path.exists(a.power_table):
        power = PowerTable.load(a.power_table)
    csvw = TelemetryCsv(a.log, power, a.board_w) if a.log else None
    link = make_link(a, on_async=lambda m: _print_msg(m, a.raw))
    try:
        rc = _run(a, link)
        if a.seconds and csvw:
            n = link.log_telemetry(a.seconds, csvw)
            print("logged %d telemetry rows to %s" % (n, a.log))
        return rc
    except (RecError, TimeoutError, ValueError) as e:
        print("error:", e, file=sys.stderr)
        return 1
    finally:
        if csvw:
            csvw.close()
        link.tp.close()


def _run(a, link):
    c = a.cmd
    show = lambda r: [print(m.text if a.raw else pretty(m)) for m in r.pre
                      if m.kind == "P"] + [_print_msg(r.msg, a.raw)]
    simple = {"stat": cmd_stat, "cfg": cmd_cfg, "res": cmd_res, "ddr": cmd_ddr,
              "stop": cmd_stop, "pfail": cmd_pfail, "abort": cmd_abort, "mon": cmd_mon,
              "dtest": cmd_dtest, "smart-q": cmd_smart_q, "prof-q": cmd_prof_q,
              "format": cmd_format}
    if c == "ping":
        lost, lat = 0, []
        for _ in range(a.count):
            try:
                m, dt = link.ping()
                lat.append(dt)
                if a.count == 1:
                    _print_msg(m, a.raw)
            except (TimeoutError, DeviceError, ParseError):
                lost += 1
        if a.count > 1:
            print("ping: %d sent, %d lost, mean %.3f ms, max %.3f ms" % (
                a.count, lost, 1e3 * sum(lat) / max(1, len(lat)), 1e3 * max(lat or [0])))
        return 1 if lost else 0
    if c in simple:
        show(link.command(simple[c]()))
    elif c == "tel":
        show(link.command(cmd_tel(a.ms)))
    elif c == "smart":
        show(link.command(cmd_smart(a.ms)))
    elif c == "mode":
        show(link.command(cmd_mode(a.which.upper() == "BP")))
    elif c == "rate":
        show(link.command(cmd_rate(rate_to_step(parse_rate(a.rate)))))
    elif c == "sid":
        show(link.command(cmd_sid(a.sid)))
    elif c == "ddrsz":
        show(link.command(cmd_ddrsz(mib_to_ddrsz(parse_size(a.size) / (1 << 20)))))
    elif c == "rec":
        if a.sid is not None:
            link.command(cmd_sid(a.sid))
        if a.rate:
            link.command(cmd_rate(rate_to_step(parse_rate(a.rate))))
        if a.length:
            link.command(cmd_len(bytes_to_slots(parse_size(a.length))))
        show(link.command(cmd_rec(a.profile)))
    elif c == "prof":
        with open(a.file) as f:
            segs = parse_profile_text(f.read())
        link.upload_profile(segs, a.rep, a.end == "stop")
        print("profile uploaded: %d segments, rep=%d, end=%s" % (len(segs), a.rep, a.end))
    elif c == "read":
        w = a.what.upper()
        if a.sid is not None:
            link.command(cmd_read_sid(a.sid))
        show(link.command(cmd_read_last() if w == "LAST" else cmd_read_end() if w == "END"
                          else cmd_read(bytes_to_slots(parse_size(a.what)))))
    elif c == "memtest":
        show(link.command(cmd_memtest(None if not a.size else
                                      mib_to_ddrsz(parse_size(a.size) / (1 << 20)))))
    elif c == "raw":
        show(link.command(a.line))
    return 0


if __name__ == "__main__":
    sys.exit(main())

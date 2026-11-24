#!/usr/bin/env python3
"""Block format (STAND/RECORDER_SPEC.md section 3) encoder/decoder and CRC vectors.

Standard library only.  Used by recorder.py, by the tests and to generate
sim/vectors/crc_vectors.hex (consumed by tb_rec_framer).

CLI:
  rec_decode.py decode FILE [--hexfmt auto|bin|hex]   decode blocks, print table
  rec_decode.py gen-crc-vectors [OUT]                  write crc_vectors.hex
  rec_decode.py check-crc-vectors [FILE]               verify file against zlib
"""
import argparse
import os
import struct
import sys
import zlib

BLOCK_BYTES = 4096
BEAT_BYTES = 16
BEATS = 256
HDR_BEATS = 3                 # beats 0..2
PAY_FIRST, PAY_LAST = 3, 254  # payload beats
PAYLOAD_BEATS = 252
MAGIC = 0x4352564E            # "NVRC"
MAGIC_END = 0x42444E45        # "ENDB"
VERSION = 1
SLOT_BLOCKS = 32
CRC_SPAN = 4080               # bytes 0..4079
M32 = 0xFFFFFFFF

FLAG_FIRST, FLAG_LAST, FLAG_PAD, FLAG_TRUNC, FLAG_PF, FLAG_DROP, FLAG_PROF, FLAG_BP = (
    1 << i for i in range(8))
FLAG_NAMES = ("FIRST", "LAST", "PAD", "TRUNC", "PF", "DROP", "PROF", "BP")

DEFAULT_VECTORS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "sim", "vectors", "crc_vectors.hex")


class BlockError(ValueError):
    pass


def crc32(data, init=0):
    """CRC-32 zlib (reflected 0x04C11DB7, init/xorout 0xFFFFFFFF)."""
    return zlib.crc32(data, init) & M32


def crc32_bitwise(data):
    """Independent bit-serial reference (for cross-checking zlib)."""
    c = M32
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ (0xEDB88320 if c & 1 else 0)
    return c ^ M32


def payload_beat(n):
    """Pattern beat for n = pos*256 + j: {~n, n^A5A5A5A5, n*7, n}, little endian."""
    n &= M32
    return struct.pack("<4I", n, (n * 7) & M32, n ^ 0xA5A5A5A5, (~n) & M32)


def flags_str(f):
    return "|".join(nm for i, nm in enumerate(FLAG_NAMES) if f >> i & 1) or "-"


def build_block(pos, seq=None, sid=0, ts=0, flags=0, seg=0, rep=0, valid=PAYLOAD_BEATS,
                lost=0, ddr_kib=0, ver=VERSION, magic=MAGIC, magic_end=MAGIC_END):
    """Build one 4096-byte block.  seq defaults to pos + lost (seq - lost = pos)."""
    if seq is None:
        seq = pos + lost
    b = bytearray(BLOCK_BYTES)
    struct.pack_into("<IIQ", b, 0, magic, sid & M32, pos)
    struct.pack_into("<QQ", b, 16, seq, ts)
    struct.pack_into("<BBHHHII", b, 32, ver, flags, seg, rep, valid, lost, ddr_kib)
    for j in range(PAYLOAD_BEATS):
        beat = PAY_FIRST + j
        # payload beat index within the block is the beat number (0..255)
        b[beat * 16:beat * 16 + 16] = payload_beat(pos * BEATS + beat)
    struct.pack_into("<II", b, 4080, seq & M32, pos & M32)
    struct.pack_into("<I", b, 4088, crc32(bytes(b[:CRC_SPAN])))
    struct.pack_into("<I", b, 4092, magic_end)
    return bytes(b)


def decode_block(blk, check_payload=True):
    """Decode and check one block.  Returns dict with fields and 'errors' list."""
    if len(blk) != BLOCK_BYTES:
        raise BlockError("block length %d != 4096" % len(blk))
    magic, sid, pos = struct.unpack_from("<IIQ", blk, 0)
    seq, ts = struct.unpack_from("<QQ", blk, 16)
    ver, flags, seg, rep, valid, lost, ddr_kib = struct.unpack_from("<BBHHHII", blk, 32)
    seq_lo, pos_lo, crc, magic_end = struct.unpack_from("<4I", blk, 4080)
    d = dict(magic=magic, sid=sid, pos=pos, seq=seq, ts=ts, ver=ver, flags=flags, seg=seg,
             rep=rep, valid=valid, lost=lost, ddr_kib=ddr_kib, seq_lo=seq_lo,
             pos_lo=pos_lo, crc=crc, magic_end=magic_end)
    err = []
    if magic != MAGIC:
        err.append("magic")
    if magic_end != MAGIC_END:
        err.append("magic_end")
    if ver != VERSION:
        err.append("ver")
    if valid > PAYLOAD_BEATS:
        err.append("valid")
    if seq - lost != pos:
        err.append("seq_lost_pos")
    if seq_lo != seq & M32:
        err.append("seq_lo")
    if pos_lo != pos & M32:
        err.append("pos_lo")
    calc = crc32(blk[:CRC_SPAN])
    d["crc_calc"] = calc
    if crc != calc:
        err.append("crc")
    bad = []
    if check_payload:
        for beat in range(PAY_FIRST, PAY_LAST + 1):
            if blk[beat * 16:beat * 16 + 16] != payload_beat(pos * BEATS + beat):
                bad.append(beat)
        if bad:
            err.append("payload")
    d["payload_bad_beats"] = bad
    d["errors"] = err
    return d


# ---------------------------------------------------------------------------
# File reading
# ---------------------------------------------------------------------------
def parse_hex_text(text):
    """Parse $readmemh-style text into bytes.

    Tokens of 2/8/16/32 hex digits (all the same width) are words written
    MSB-first (as $readmemh of a 8/32/64/128-bit memory); they are emitted
    little-endian, i.e. the word is memory byte 0 = bits [7:0].
    Comments: // to end of line, /* */ not supported; '@addr' lines ignored.
    """
    toks = []
    for ln in text.splitlines():
        ln = ln.split("//")[0].strip()
        if not ln or ln.startswith("@"):
            continue
        toks += ln.replace("_", "").split()
    if not toks:
        return b""
    w = len(toks[0])
    if w not in (2, 8, 16, 32) or any(len(t) != w for t in toks):
        raise BlockError("hex file: tokens must all be 2/8/16/32 hex digits wide")
    out = bytearray()
    for t in toks:
        out += int(t, 16).to_bytes(w // 2, "little")
    return bytes(out)


def read_blocks(path, fmt="auto"):
    with open(path, "rb") as f:
        raw = f.read()
    if fmt == "auto":
        head = raw[:4096]
        try:
            head.decode("ascii")
            txt = all(c in b"0123456789abcdefABCDEF_ \t\r\n/@" or 32 <= c < 127
                      for c in head) and b"\x00" not in head
        except UnicodeDecodeError:
            txt = False
        fmt = "hex" if txt else "bin"
    data = parse_hex_text(raw.decode("ascii")) if fmt == "hex" else raw
    if len(data) % BLOCK_BYTES:
        raise BlockError("size %d is not a multiple of 4096" % len(data))
    return [data[i:i + BLOCK_BYTES] for i in range(0, len(data), BLOCK_BYTES)]


def chain_check(decoded, sid=None):
    """Chain-level checks over decoded blocks. Returns list of strings."""
    out = []
    for i, d in enumerate(decoded):
        if i and d["pos"] != decoded[i - 1]["pos"] + 1:
            out.append("pos gap before block %d" % i)
        if i and d["seq"] < decoded[i - 1]["seq"]:
            out.append("seq decreases at block %d" % i)
        if sid is not None and d["sid"] != sid:
            out.append("sid mismatch at block %d" % i)
    if decoded and not decoded[-1]["flags"] & FLAG_LAST:
        out.append("no LAST at end of chain (recording interrupted)")
    return out


# ---------------------------------------------------------------------------
# crc_vectors.hex
# ---------------------------------------------------------------------------
def crc_vector_messages():
    """Deterministic 4080-byte messages (255 beats of 16 B)."""
    msgs = [("zeros", bytes(CRC_SPAN)),
            ("ones", b"\xff" * CRC_SPAN),
            ("count", bytes(i & 0xFF for i in range(CRC_SPAN)))]
    for pos in (0, 1, 0x12345):
        msgs.append(("block_pos%X" % pos, build_block(pos, sid=0x692D3A00, ts=1234 + pos,
                                                       flags=FLAG_FIRST if pos == 0 else 0,
                                                       ddr_kib=77)[:CRC_SPAN]))
    x = 0x1234ABCD
    for k in range(3):
        buf = bytearray()
        while len(buf) < CRC_SPAN:       # xorshift32, deterministic
            x ^= (x << 13) & M32
            x ^= x >> 17
            x ^= (x << 5) & M32
            buf += struct.pack("<I", x)
        msgs.append(("random%d" % k, bytes(buf[:CRC_SPAN])))
    return msgs


def gen_crc_vectors_text():
    """File layout (for $readmemh into a 128-bit x (256*N) memory):
    vector v occupies 256 lines: lines 0..254 = beats 0..254 (128-bit, MSB first,
    i.e. byte 15 first; byte 0 = bits[7:0]), line 255 = {96'h0, crc32}.
    '//' comment lines carry names.  Number of vectors N = len(crc_vector_messages())."""
    lines = ["// crc_vectors.hex - generated by tools/rec_decode.py gen-crc-vectors; do not edit",
             "// each vector = 256 lines x 128 bit: beats 0..254 of a 4080-byte message,",
             "// then {96'h0, crc32(zlib)}; byte 0 of a beat = bits [7:0]"]
    msgs = crc_vector_messages()
    lines.append("// vectors: %d" % len(msgs))
    for name, m in msgs:
        lines.append("// vector %s" % name)
        for i in range(0, CRC_SPAN, 16):
            lines.append(m[i:i + 16][::-1].hex().upper())
        lines.append("%032X" % zlib.crc32(m))
    return "\n".join(lines) + "\n"


def check_crc_vectors(path):
    """Return list of problems (empty = file is consistent with zlib.crc32)."""
    with open(path) as f:
        data = f.read()
    toks = [ln.strip() for ln in data.splitlines()
            if ln.strip() and not ln.strip().startswith("//")]
    probs = []
    if len(toks) % 256:
        return ["line count %d not multiple of 256" % len(toks)]
    for v in range(len(toks) // 256):
        rows = toks[v * 256:(v + 1) * 256]
        msg = b"".join(bytes.fromhex(r)[::-1] for r in rows[:255])
        want = int(rows[255], 16)
        if want != zlib.crc32(msg) & M32:
            probs.append("vector %d: stored %08X != zlib %08X" % (v, want, zlib.crc32(msg)))
    return probs


# ---------------------------------------------------------------------------
def _cmd_decode(a):
    blocks = read_blocks(a.file, a.hexfmt)
    dec = [decode_block(b) for b in blocks]
    print("%d block(s)" % len(dec))
    nbad = 0
    for i, d in enumerate(dec):
        e = ",".join(d["errors"]) or "ok"
        nbad += bool(d["errors"])
        print("#%d pos=%d seq=%d lost=%d sid=%08X ts=%d seg=%d rep=%d valid=%d ddr_kib=%d "
              "flags=%s crc=%08X %s" % (i, d["pos"], d["seq"], d["lost"], d["sid"], d["ts"],
                                        d["seg"], d["rep"], d["valid"], d["ddr_kib"],
                                        flags_str(d["flags"]), d["crc"], e))
    for s in chain_check(dec):
        print("chain: " + s)
    return 1 if nbad else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("decode")
    p.add_argument("file")
    p.add_argument("--hexfmt", choices=("auto", "bin", "hex"), default="auto")
    p = sub.add_parser("gen-crc-vectors")
    p.add_argument("out", nargs="?", default=DEFAULT_VECTORS)
    p = sub.add_parser("check-crc-vectors")
    p.add_argument("file", nargs="?", default=DEFAULT_VECTORS)
    a = ap.parse_args(argv)
    if a.cmd == "decode":
        return _cmd_decode(a)
    if a.cmd == "gen-crc-vectors":
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, "w", newline="\n") as f:
            f.write(gen_crc_vectors_text())
        print("wrote", os.path.abspath(a.out))
        return 0
    probs = check_crc_vectors(a.file)
    for s in probs:
        print(s)
    print("OK" if not probs else "FAIL")
    return 1 if probs else 0


if __name__ == "__main__":
    sys.exit(main())

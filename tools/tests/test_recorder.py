"""recorder.py tests: commands, parsing of every spec message, units, CSV, power, fake device.
Run: python tools/recorder.py --selftest   (or pytest tools/tests)"""
import contextlib
import csv
import io
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import recorder as rc  # noqa: E402

T_GOLDEN = ("$T v=01 t=0001A2F0 st=2 md=1 cok=1 uok=1 cs=5 ce=00 cd=00 fl=00 lk=9 cal=1 "
            "stp=2CCCCCCC seg=00 rep=0001 src=000000000400 wr=000000000400 drp=00000000 "
            "ddr=00100000 ddrm=00200000 bpm=0000000000 rng=00000010 ssd=0000000000400000 "
            "cmd=00000020 cer=00000000 lat=00001000 vb=000000000000 ve=00000000 tk=0136 "
            "cw=00 smo=0003 sme=0000 xt=A1B xvi=555 xva=999 xvb=555")

RP_LINE = ("!RP src=1 inb=0 lat=0 lov=0 pnd=000000100000 pdd=00100000 prg=00000200 pad=00000003 "
           "t1=0000000000000010 t2=0000000000000100 t3=0000000000100000 t4=0000000000200000 "
           "t5=0000000000300000 t6=0000000000400000 ok=1 rtd=000186A0 ssd=0000000000400000 "
           "lpos=00000003FF")

GOLDEN = {   # one valid line for every device message in spec 2.3-2.6
    "OK PING ver=02 bld=20261201 cal=1 uok=1": ("OK", "PING"),
    "OK CFG md=1 bp=0 stp=2CCCCCCC len=00000020 sid=692D3A00 ddr=0400 tel=03E8 sm=03E8 pn=02 "
    "pr=0004 pe=1 rsid=692D3A00": ("OK", "CFG"),
    "OK TEL tel=0064": ("OK", "TEL"),
    "OK SMART sm=03E8": ("OK", "SMART"),
    "OK SMART cw=00 tk=0136 sp=64 us=05 dur=0000000000001000 duw=0000000000002000 "
    "pc=00000010 poh=00000100 uns=00000002 me=00000000 wt=00000130 ct=00000000 t1=0000 "
    "ok=0010 er=0000 st=0000 rtd=000186A0": ("OK", "SMART"),
    "OK MODE bp=1": ("OK", "MODE"),
    "OK RATE stp=2CCCCCCC": ("OK", "RATE"),
    "OK LEN len=00000020": ("OK", "LEN"),
    "OK SID sid=692D3A00": ("OK", "SID"),
    "OK DDRSZ ddr=0020": ("OK", "DDRSZ"),
    "OK PROF pn=00": ("OK", "PROF"),
    "OK PROF i=01": ("OK", "PROF"),
    "OK PROF pr=0004": ("OK", "PROF"),
    "OK PROF pe=1": ("OK", "PROF"),
    "OK PROF pn=02 pr=0004 pe=1": ("OK", "PROF"),
    "P i=00 stp=2CCCCCCC dur=00B2D05E00": ("P", "P"),
    "OK REC sid=692D3A00": ("OK", "REC"),
    "OK STOP": ("OK", "STOP"),
    "OK PFAIL": ("OK", "PFAIL"),
    "OK ABORT": ("OK", "ABORT"),
    "OK READ": ("OK", "READ"),
    "OK READ rsid=692D3A00": ("OK", "READ"),
    "OK MEMTEST": ("OK", "MEMTEST"),
    "OK MEMTEST mib=0400": ("OK", "MEMTEST"),
    "OK DDR cal=1 ddr=00100000 ddrm=00200000 mte=00000000 mta=00000000": ("OK", "DDR"),
    "OK": ("OK", ""),
    "ERR 0C": ("ERR", ""),
    "!B ver=02 cal=1 uok=1": ("EV", "B"),
    "!S st=2 md=1 t=0001A2F0": ("EV", "S"),
    "!E ce=01 cd=02 fl=04 st=2": ("EV", "E"),
    "!RR sid=692D3A00 wr=000000000400 drp=00000000 pad=00000003 trc=00000001 ddrm=00200000 "
    "ssd=0000000000400000 cmd=00000020 cer=00000000 lat=00001000 dur=0000000011E1A300":
        ("EV", "RR"),
    RP_LINE: ("EV", "RP"),
    "!RV vb=000000400000 ok=00000003FF bad=00000000 crc=00000000 pe=00000000 ooo=00000000 "
    "prt=00000000 fbp=FFFFFFFFFF mvp=FFFFFFFFFF lsq=0000000000000000 lts=0000000000000000 "
    "lst=00000000 pad=00000003 fep=FFFFFFFFFF feb=00 exp=00000000 got=00000000 end=1 ovf=0 "
    "lfl=02 dur=0000000000100000": ("EV", "RV"),
    "!RF ms=000004B0 ok=1": ("EV", "RF"),
    "!RM byt=20000000 err=00000000 adr=00000000 dur=0000000005F5E100": ("EV", "RM"),
    "!RD wr=000000000400 drp=00000000 he=00000000 dur=0000000005F5E100": ("EV", "RD"),
    T_GOLDEN: ("T", "T"),
}


class TestUnits(unittest.TestCase):
    def test_spec_example_step(self):
        # spec 2.7 example prints 2CCCCCCC for 280 MB/s; round() of the exact value is ...CCD
        self.assertAlmostEqual(rc.step_to_rate(0x2CCCCCCC), 280e6, delta=1)
        self.assertIn(rc.rate_to_step(280e6), (0x2CCCCCCC, 0x2CCCCCCD))
        self.assertEqual(rc.rate_to_step(0), 0)

    def test_rate_roundtrip(self):
        for r in (1e6, 50e6, 100e6, 280e6, 400e6, 800e6, 1.6e9 * 0.999):
            st = rc.rate_to_step(r)
            self.assertLessEqual(abs(rc.step_to_rate(st) - r), rc.step_to_rate(1) / 2 + 1e-6)
            self.assertEqual(rc.rate_to_step(rc.step_to_rate(st)), st)

    def test_step_roundtrip_sample(self):
        for st in list(range(0, 5000, 7)) + [0x7FFFFFFF, 0xFFFFFFFE, 0xFFFFFFFF, 0x2CCCCCCC]:
            self.assertEqual(rc.rate_to_step(rc.step_to_rate(st)), st)

    def test_rate_out_of_range(self):
        with self.assertRaises(ValueError):
            rc.rate_to_step(1.7e9)
        with self.assertRaises(ValueError):
            rc.rate_to_step(-1)

    def test_time(self):
        self.assertEqual(rc.us_to_cycles(30e6), 0xB2D05E00)         # spec 2.7: 30 s
        self.assertEqual(rc.us_to_cycles(0.5e6), 0x2FAF080)         # 0.5 s
        self.assertEqual(rc.cycles_to_us(0xB2D05E00), 30e6)
        for us in (0.01, 1, 123.45, 1e6, 3.5e9):
            self.assertAlmostEqual(rc.cycles_to_us(rc.us_to_cycles(us)), us, delta=0.005)

    def test_slots_and_sizes(self):
        self.assertEqual(rc.bytes_to_slots(1), 1)
        self.assertEqual(rc.bytes_to_slots(131072), 1)
        self.assertEqual(rc.bytes_to_slots(131073), 2)
        self.assertEqual(rc.bytes_to_slots(4 << 20), 32)
        self.assertEqual(rc.parse_size("4MiB"), 4 << 20)
        self.assertEqual(rc.parse_rate("280MB/s"), 280e6)
        self.assertEqual(rc.parse_rate("1GiB/s"), 1 << 30)
        self.assertAlmostEqual(rc.parse_duration("500ms"), 0.5)
        self.assertAlmostEqual(rc.parse_duration("10us"), 1e-5)
        with self.assertRaises(ValueError):
            rc.parse_rate("fast")
        with self.assertRaises(ValueError):
            rc.parse_size("3furlongs")

    def test_xadc(self):
        self.assertAlmostEqual(rc.xadc_temp_c(0xA1B), 45.2, delta=0.1)
        self.assertAlmostEqual(rc.xadc_volt(0x555), 1.0, delta=0.001)
        for t in (-10, 25, 85):
            self.assertAlmostEqual(rc.xadc_temp_c(rc.temp_c_to_xadc(t)), t, delta=0.15)
        self.assertAlmostEqual(rc.xadc_volt(rc.volt_to_xadc(1.8)), 1.8, delta=0.001)


class TestCommands(unittest.TestCase):
    def test_golden_lines(self):
        g = rc
        pairs = [
            (g.cmd_ping(), "PING"), (g.cmd_tel(100), "TEL 0064"), (g.cmd_tel(0), "TEL 0000"),
            (g.cmd_smart(1000), "SMART 03E8"), (g.cmd_mode(True), "MODE BP"),
            (g.cmd_mode(False), "MODE RT"), (g.cmd_rate(0x2CCCCCCC), "RATE 2CCCCCCC"),
            (g.cmd_len(0x20), "LEN 00000020"), (g.cmd_sid(0x692D3A00), "SID 692D3A00"),
            (g.cmd_ddrsz(32), "DDRSZ 0020"),
            (g.cmd_prof_add(0x2CCCCCCC, 0xB2D05E00), "PROF ADD 2CCCCCCC 00B2D05E00"),
            (g.cmd_prof_add(0, 0x2FAF080), "PROF ADD 00000000 0002FAF080"),
            (g.cmd_prof_set(3, 1, 2), "PROF SET 03 00000001 0000000002"),
            (g.cmd_prof_rep(4), "PROF REP 0004"), (g.cmd_prof_end(True), "PROF END STOP"),
            (g.cmd_prof_end(False), "PROF END HOLD"), (g.cmd_rec(), "REC"),
            (g.cmd_rec(True), "REC PROF"), (g.cmd_read(0x40), "READ 00000040"),
            (g.cmd_read_sid(0xBEEF), "READ SID 0000BEEF"), (g.cmd_read_last(), "READ LAST"),
            (g.cmd_read_end(), "READ END"), (g.cmd_format(), "FORMAT YES"),
            (g.cmd_memtest(), "MEMTEST"), (g.cmd_memtest(0x400), "MEMTEST 0400"),
            (g.cmd_stat(), "STAT?"), (g.cmd_cfg(), "CFG?"), (g.cmd_res(), "RES?"),
            (g.cmd_prof_q(), "PROF?"), (g.cmd_ddr(), "DDR?"), (g.cmd_smart_q(), "SMART?"),
            (g.cmd_prof_clr(), "PROF CLR"), (g.cmd_stop(), "STOP"), (g.cmd_pfail(), "PFAIL"),
            (g.cmd_abort(), "ABORT"), (g.cmd_mon(), "MON"), (g.cmd_dtest(), "DTEST"),
        ]
        for got, want in pairs:
            self.assertEqual(got, want)

    def test_longest_lines_fit(self):
        for ln in (rc.cmd_prof_set(63, 0xFFFFFFFF, rc.DUR_MAX),
                   rc.cmd_prof_add(0xFFFFFFFF, rc.DUR_MAX)):
            self.assertLessEqual(len(ln), 64)
            self.assertRegex(ln, r"^[A-Z0-9? ]+$")

    def test_range_errors(self):
        for f in (lambda: rc.cmd_tel(5), lambda: rc.cmd_smart(50), lambda: rc.cmd_ddrsz(0),
                  lambda: rc.cmd_ddrsz(0x401), lambda: rc.cmd_prof_add(1, 0),
                  lambda: rc.cmd_read(0), lambda: rc.cmd_rate(1 << 32),
                  lambda: rc.cmd_prof_rep(0x10000), lambda: rc.cmd_prof_add(0, 1 << 40),
                  lambda: rc.check_line("X" * 65)):
            with self.assertRaises(ValueError):
                f()


class TestParsing(unittest.TestCase):
    def test_every_message_parses_strict(self):
        for line, (kind, name) in GOLDEN.items():
            m = rc.parse_line(line)
            self.assertEqual((m.kind, m.name), (kind, name), line)

    def test_telemetry_values_and_length(self):
        # spec 2.5 says "about 290 chars"; the listed fields actually give 355 (see notes)
        self.assertEqual(len(T_GOLDEN), 3 + sum(len(k) + 1 + w + 1 for k, w in rc.T_SPEC) - 1)
        m = rc.parse_line(T_GOLDEN)
        self.assertEqual(m["t"], 0x1A2F0)
        self.assertEqual(m["stp"], 0x2CCCCCCC)
        self.assertEqual(m["ssd"], 0x400000)
        self.assertEqual(m["lk"], 0x9)
        d = rc.telemetry_derived(m)
        self.assertAlmostEqual(d["t_s"], 107.248)
        self.assertEqual((d["state"], d["mode"], d["core_state"]), ("RUN", "REC", "RUN"))
        self.assertEqual((d["link_up"], d["gen2"], d["lanes"]), (1, 0, 4))   # 0x9: b3:2 = 2 = log2(4 lanes)
        self.assertAlmostEqual(d["rate_Bps"], 280e6, delta=1)
        self.assertEqual(d["src_bytes"], 0x400 * 4096)
        self.assertAlmostEqual(d["xadc_temp_C"], 45.2, delta=0.1)
        self.assertAlmostEqual(d["smart_temp_C"], 0x136 - 273.15)
        self.assertIn("RUN/REC", rc.pretty(m))

    def test_format_telemetry_matches_golden(self):
        self.assertEqual(rc.format_telemetry(dict(rc.parse_line(T_GOLDEN).f)), T_GOLDEN)

    def test_err_codes(self):
        for code in (1, 2, 3, 4, 5, 6, 8, 9, 0xA, 0xB, 0xC, 0xD):
            m = rc.parse_line("ERR %02X" % code)
            self.assertEqual(m["code"], code)
            self.assertIn("ERR %02X" % code, rc.pretty(m))

    def test_pretty_units(self):
        m = rc.parse_line(RP_LINE)
        self.assertAlmostEqual(rc.result_derived(m)["t6_s"], 0x400000 * 5e-9)
        self.assertIn("t6_s", rc.pretty(m))
        m = rc.parse_line("P i=00 stp=2CCCCCCC dur=00B2D05E00")
        self.assertIn("30.000 s", rc.pretty(m))
        self.assertIn("280.000 MB/s", rc.pretty(m))
        self.assertIn("bld=20261201",
                      rc.pretty(rc.parse_line("OK PING ver=02 bld=20261201 cal=1 uok=1")))

    def test_blank_and_tolerant(self):
        self.assertIsNone(rc.parse_line("  \r\n"))
        m = rc.parse_line("!X foo=1")                     # unknown event: parsed, not checked
        self.assertEqual((m.kind, m.name, m["foo"]), ("EV", "X", 1))
        m = rc.parse_line("OK FORMAT whatever=12")          # undefined reply fields: not checked
        self.assertEqual(m["whatever"], 0x12)
        self.assertEqual(rc.parse_line("OK PING ver=02 bld=20261201 cal=1 uok=1\r\n").name,
                         "PING")

    def test_negative_controls_rejected(self):
        bad = {
            "T short field": T_GOLDEN.replace("t=0001A2F0", "t=1A2F0"),
            "T long field": T_GOLDEN.replace("cs=5", "cs=05"),
            "T missing key": T_GOLDEN.replace(" cer=00000000", ""),
            "T swapped order": T_GOLDEN.replace("st=2 md=1", "md=1 st=2"),
            "T non-hex": T_GOLDEN.replace("stp=2CCCCCCC", "stp=2CCCCCCZ"),
            "T extra key": T_GOLDEN + " zz=1",
            "T version": T_GOLDEN.replace("v=01", "v=02"),
            "T no equals": T_GOLDEN.replace("md=1", "md1"),
            "T dup key": T_GOLDEN + " t=00000001",
            "PING width": "OK PING ver=2 bld=20261201 cal=1 uok=1",
            "PING extra": "OK PING ver=02 bld=20261201 cal=1 uok=1 x=1",
            "PING missing": "OK PING ver=02 bld=20261201 cal=1",
            "TEL width": "OK TEL tel=64",
            "PROF unknown form": "OK PROF pn=00 pe=1",
            "ERR 3 digits": "ERR 00C",
            "ERR no code": "ERR",
            "ERR non hex": "ERR ZZ",
            "!S width": "!S st=02 md=1 t=0001A2F0",
            "!RP missing": RP_LINE.replace(" rtd=000186A0", ""),
            "!RP order": RP_LINE.replace("src=1 inb=0", "inb=0 src=1"),
            "!RP width": RP_LINE.replace("ok=1", "ok=01"),
            "!RV missing": "!RV vb=000000400000",
            "P width": "P i=0 stp=2CCCCCCC dur=00B2D05E00",
            "garbage": "HELLO WORLD",
            "lowercase kw": "ok PING",
        }
        for name, line in bad.items():
            with self.assertRaises(rc.ParseError, msg=name):
                rc.parse_line(line)
        # non-strict mode accepts width deviations (but not structural garbage)
        self.assertIsNotNone(rc.parse_line("OK TEL tel=64", strict=False))
        with self.assertRaises(rc.ParseError):
            rc.parse_line("HELLO", strict=False)


class TestPower(unittest.TestCase):
    @staticmethod
    def table():
        rows = []
        for mode, rates in (("IDLE", [0]), ("MON", [0]), ("REC", [100e6, 400e6, 800e6]),
                            ("READ", [887e6])):
            for r in rates:
                for tj in (25, 85):
                    rows.append(dict(mode=mode, rate_Bps=r, tj_C=tj, p_static_W=0.2 + tj / 1000,
                                     p_dyn_W=0.5 + (r / 1e9), p_vccint_W=0, p_vccaux_W=0,
                                     p_vcco_W=0, p_mgt_W=0))
        return rc.PowerTable(rows)

    def test_interpolation(self):
        t = self.table()
        self.assertAlmostEqual(t.estimate("REC", 100e6, 25), 0.225 + 0.6)
        self.assertAlmostEqual(t.estimate("REC", 800e6, 85), 0.285 + 1.3)
        self.assertAlmostEqual(t.estimate("REC", 250e6, 25), 0.225 + 0.5 + 0.25)
        self.assertAlmostEqual(t.estimate("REC", 100e6, 55), (0.225 + 0.285) / 2 + 0.6)
        self.assertAlmostEqual(t.estimate("REC", 5e9, 200), t.estimate("REC", 800e6, 85))
        self.assertAlmostEqual(t.estimate("REC", 0, -40), t.estimate("REC", 100e6, 25))
        self.assertAlmostEqual(t.estimate_md(5, 123e6, 25), t.estimate("MON", 0, 25))
        self.assertAlmostEqual(t.estimate_md(3, 1, 25), t.estimate("READ", 887e6, 25))

    def test_negative_controls(self):
        with self.assertRaises(KeyError):
            self.table().estimate("WARP", 0, 25)
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, "t.csv")
            with open(p, "w") as f:
                f.write("mode,rate_Bps\nREC,1\n")
            with self.assertRaises(ValueError):
                rc.PowerTable.load(p)

    def test_load_csv(self):
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, "t.csv")
            with open(p, "w") as f:
                f.write(",".join(rc.POWER_COLS) + "\nrec,100000000,25,0.2,0.5,0.4,0.2,0.1,0.0\n"
                        "REC,400000000,25,0.2,1.0,0.4,0.2,0.1,0.0\n")
            self.assertAlmostEqual(rc.PowerTable.load(p).estimate("REC", 250e6, 25), 0.2 + 0.75)

    def test_repo_table_if_present(self):
        p = os.path.join(os.path.dirname(os.path.abspath(rc.__file__)), "..", "fpga", "power",
                         "power_table.csv")
        if os.path.exists(p):
            self.assertGreater(rc.PowerTable.load(p).estimate("REC", 280e6, 50), 0)

    def test_tcl_header_matches_columns(self):
        p = os.path.join(os.path.dirname(os.path.abspath(rc.__file__)), "..", "fpga", "scripts",
                         "power_table.tcl")
        with open(p) as f:
            self.assertIn(",".join(rc.POWER_COLS), f.read())


class TestCsvAndFake(unittest.TestCase):
    @staticmethod
    def run_cli(*args):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            try:
                code = rc.main(list(args))
            except SystemExit as e:
                code = e.code
        return code, out.getvalue(), err.getvalue()

    def test_csv_columns_and_power(self):
        t = TestPower.table()
        buf = io.StringIO()
        w = rc.TelemetryCsv(buf, t, board_w=12.5)
        w.add(rc.parse_line(T_GOLDEN), host_time=1.0)
        m2 = rc.parse_line(T_GOLDEN.replace("t=0001A2F0", "t=0001A6D8")
                           .replace("ssd=0000000000400000", "ssd=0000000008400000"))
        w.add(m2, host_time=2.0)
        rows = list(csv.DictReader(io.StringIO(buf.getvalue())))
        self.assertEqual(list(rows[0].keys()), rc.CSV_COLUMNS)
        self.assertEqual(rows[0]["state"], "RUN")
        self.assertEqual(rows[0]["p_note"], rc.P_NOTE)
        self.assertEqual(rows[0]["board_w"], "12.5")
        self.assertAlmostEqual(float(rows[0]["p_fpga_est_W"]),
                               t.estimate("REC", 280e6, rc.xadc_temp_c(0xA1B)), places=2)
        self.assertEqual(rows[0]["ssd_rate_Bps"], "")
        self.assertAlmostEqual(float(rows[1]["ssd_rate_Bps"]), 0x8000000 * 1000 / 1000.0)

    def test_csv_without_table(self):
        buf = io.StringIO()
        rc.TelemetryCsv(buf).add(rc.parse_line(T_GOLDEN))
        row = list(csv.DictReader(io.StringIO(buf.getvalue())))[0]
        self.assertEqual((row["p_fpga_est_W"], row["p_note"]), ("", "NO_POWER_TABLE"))

    def test_fake_ping_stat_cfg(self):
        code, out, _ = self.run_cli("--fake", "ping")
        self.assertEqual(code, 0)
        self.assertIn("OK PING ver=2 bld=20261201", out)
        code, out, _ = self.run_cli("--fake", "ping", "--count", "20")
        self.assertIn("20 sent, 0 lost", out)
        self.assertEqual(self.run_cli("--fake", "stat")[0], 0)
        self.assertEqual(self.run_cli("--fake", "cfg")[0], 0)

    def test_fake_rec_with_csv(self):
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, "log.csv")
            code, out, err = self.run_cli("--fake", "--log", p, "--seconds", "3", "rec",
                                          "--rate", "280MB/s", "--len", "4MiB")
            self.assertEqual(code, 0, err)
            self.assertIn("!RR", out)
            self.assertIn("logged", out)
            with open(p) as f:
                self.assertGreaterEqual(len(f.read().splitlines()), 3)

    def test_fake_profile_upload_and_readback(self):
        with tempfile.TemporaryDirectory() as td:
            p = os.path.join(td, "prof.txt")
            with open(p, "w") as f:
                f.write("# demo\n280MB/s 30s\n0 500ms\n")
            code, out, err = self.run_cli("--fake", "prof", "--file", p, "--rep", "4")
            self.assertEqual(code, 0, err)
            self.assertIn("2 segments", out)

    def test_link_level_protocol(self):
        link = rc.Link(rc.FakeDevice())
        link.command(rc.cmd_tel(100))
        link.upload_profile([(rc.rate_to_step(100e6), rc.us_to_cycles(1000)),
                             (0, rc.us_to_cycles(500))], rep=3, end_stop=False)
        r = link.command(rc.cmd_cfg())
        self.assertEqual((r.msg["pn"], r.msg["pr"], r.msg["pe"], r.msg["tel"]), (2, 3, 0, 100))
        link.command(rc.cmd_rec(True))
        for _ in range(3):
            link.poll(0.5)                       # let virtual time run: data accumulates
        r = link.command(rc.cmd_stat())
        self.assertEqual(r.pre[-1].kind, "T")
        self.assertEqual(r.pre[-1]["st"], 2)
        link.command(rc.cmd_stop())
        for _ in range(3):                       # events follow the OK line
            link.poll(0.01)
        self.assertTrue(any(m.kind == "EV" and m.name == "RR" for m in link.async_log))
        r = link.command(rc.cmd_res())
        self.assertEqual(r.pre[-1].name, "RR")
        link.command(rc.cmd_read_last())
        link.poll(0.01)
        self.assertTrue(any(m.name == "RV" for m in link.async_log))

    def test_negative_controls_device_errors(self):
        link = rc.Link(rc.FakeDevice())
        cases = [("BOGUS", 0x02), ("TEL 5", 0x03), ("TEL", 0x01), ("DDRSZ 0000", 0x03),
                 ("REC PROF", 0x08), ("READ LAST", 0x0B), ("STOP", 0x04),
                 ("TEL 123456789012345678", 0x01)]
        for line, code in cases:
            with self.assertRaises(rc.DeviceError, msg=line) as cm:
                link.command(line)
            self.assertEqual(cm.exception.code, code, line)
        with self.assertRaises(ValueError):             # 65 chars rejected client side
            link.command("X" * 65)
        dev = rc.FakeDevice()
        dev.write(("X" * 65 + "\n").encode())
        self.assertEqual(dev.out[-1], "ERR 0A")
        for _ in range(64):
            link.command(rc.cmd_prof_add(1, 1))
        with self.assertRaises(rc.DeviceError) as cm:
            link.command(rc.cmd_prof_add(1, 1))
        self.assertEqual(cm.exception.code, 9)
        link.command(rc.cmd_rec(True))
        with self.assertRaises(rc.DeviceError) as cm:   # RATE in RUN with a profile
            link.command(rc.cmd_rate(5))
        self.assertEqual(cm.exception.code, 4)
        link.command(rc.cmd_stop())

    def test_read_sid_memtest_replies_and_rv(self):
        link = rc.Link(rc.FakeDevice())
        self.assertEqual(link.command("READ SID 692D3A00").msg["rsid"], 0x692D3A00)
        self.assertEqual(link.command("MEMTEST 0020").msg["mib"], 0x20)
        self.assertEqual(link.command("MEMTEST").msg["mib"], 0x400)
        for _ in range(2):
            link.poll(0.01)
        with self.assertRaises(rc.ParseError):          # negative: bare form with wrong width
            rc.parse_line("OK MEMTEST mib=20")
        for lk, lanes in ((0x1, 1), (0x5, 2), (0x9, 4), (0xD, 8)):
            m = rc.parse_line(T_GOLDEN.replace("lk=9", "lk=%X" % lk))
            self.assertEqual(rc.telemetry_derived(m)["lanes"], lanes)

    def test_cli_error_exit_code(self):
        code, _, err = self.run_cli("--fake", "read", "LAST")
        self.assertEqual(code, 1)
        self.assertIn("ERR 0B", err)
        self.assertEqual(self.run_cli("--fake", "rate", "5GB/s")[0], 1)

    def test_convert_cli(self):
        _, out, _ = self.run_cli("convert", "rate", "280MB/s")
        self.assertIn("step=2CCCCC", out)
        _, out, _ = self.run_cli("convert", "us", "30000000")
        self.assertIn("00B2D05E00", out)

    def test_profile_text(self):
        segs = rc.parse_profile_text("280MB/s 30s\n0 0.5s # pause\n")
        self.assertEqual(segs[1], (0, 0x2FAF080))
        self.assertAlmostEqual(rc.step_to_rate(segs[0][0]), 280e6, delta=1)
        for bad in ("", "280MB/s", "280MB/s 0s", "1 1s\n" * 65, "9GB/s 1s"):
            with self.assertRaises(ValueError, msg=repr(bad)):
                rc.parse_profile_text(bad)

    def test_serial_is_lazy(self):
        saved = sys.modules.get("serial", "absent")
        sys.modules["serial"] = None            # makes 'import serial' raise ImportError
        try:
            with self.assertRaises(rc.RecError):
                rc.SerialTransport("COM1")
        finally:
            if saved == "absent":
                del sys.modules["serial"]
            else:
                sys.modules["serial"] = saved


if __name__ == "__main__":
    unittest.main()

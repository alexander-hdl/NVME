"""Block format / CRC tests (spec 3.1).  Run: python tools/recorder.py --selftest  or  pytest tools/tests"""
import os
import struct
import sys
import tempfile
import unittest
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import rec_decode as rd  # noqa: E402


class TestCrc(unittest.TestCase):
    def test_known_check_value(self):
        self.assertEqual(rd.crc32(b"123456789"), 0xCBF43926)

    def test_equals_zlib_and_bitwise(self):
        for name, msg in rd.crc_vector_messages():
            self.assertEqual(rd.crc32(msg), zlib.crc32(msg) & 0xFFFFFFFF, name)
            self.assertEqual(rd.crc32_bitwise(msg), zlib.crc32(msg) & 0xFFFFFFFF, name)

    def test_negative_control_corrupted_vector_differs(self):
        for name, msg in rd.crc_vector_messages():
            bad = bytearray(msg)
            bad[len(bad) // 2] ^= 0x01
            self.assertNotEqual(rd.crc32(bytes(bad)), rd.crc32(msg), name)

    def test_vector_file_matches_zlib(self):
        self.assertEqual(rd.check_crc_vectors(rd.DEFAULT_VECTORS), [])

    def test_vector_file_is_current(self):
        with open(rd.DEFAULT_VECTORS) as f:
            self.assertEqual(f.read(), rd.gen_crc_vectors_text())

    def test_negative_control_corrupted_vector_file_fails(self):
        txt = rd.gen_crc_vectors_text().splitlines()
        for target in (10, 255 + 4):         # data row of vector 0 / ... ; crc row handled below
            lines = list(txt)
            idx = [i for i, l in enumerate(lines) if not l.startswith("//")][target]
            v = int(lines[idx], 16) ^ 1
            lines[idx] = "%032X" % v
            with tempfile.NamedTemporaryFile("w", suffix=".hex", delete=False) as f:
                f.write("\n".join(lines) + "\n")
            try:
                self.assertTrue(rd.check_crc_vectors(f.name), "corruption not detected")
            finally:
                os.unlink(f.name)
        # corrupt a stored CRC row
        lines = list(txt)
        crc_rows = [i for i, l in enumerate(lines) if not l.startswith("//")][255::256]
        lines[crc_rows[0]] = "%032X" % (int(lines[crc_rows[0]], 16) ^ 0x80000000)
        with tempfile.NamedTemporaryFile("w", suffix=".hex", delete=False) as f:
            f.write("\n".join(lines) + "\n")
        try:
            self.assertEqual(len(rd.check_crc_vectors(f.name)), 1)
        finally:
            os.unlink(f.name)


class TestBlock(unittest.TestCase):
    def test_payload_beat_golden(self):
        # n=3: {~n, n^A5A5A5A5, n*7, n} little-endian, byte 0 = n[7:0]
        self.assertEqual(rd.payload_beat(3).hex(),
                         "03000000" "15000000" "a6a5a5a5" "fcffffff")

    def test_header_golden_bytes(self):
        b = rd.build_block(5, sid=0x692D3A00, ts=0x0102030405060708, flags=rd.FLAG_FIRST,
                           seg=2, rep=3, valid=252, lost=0, ddr_kib=0x11)
        self.assertEqual(b[0:8].hex(), "4e565243" "003a2d69")           # magic, sid
        self.assertEqual(b[8:16], struct.pack("<Q", 5))                   # pos
        self.assertEqual(b[16:24], struct.pack("<Q", 5))                  # seq
        self.assertEqual(b[24:32].hex(), "0807060504030201")              # ts
        self.assertEqual(b[32:40].hex(), "0101" "0200" "0300" "fc00")     # ver flags seg rep valid
        self.assertEqual(b[44:48], struct.pack("<I", 0x11))
        self.assertEqual(b[4092:4096].hex(), "454e4442")                  # ENDB
        self.assertEqual(struct.unpack_from("<I", b, 4088)[0], zlib.crc32(b[:4080]))

    def test_roundtrip_all_fields(self):
        b = rd.build_block(0x123456789, seq=0x123456789 + 7, sid=0xDEADBEEF, ts=99,
                           flags=rd.FLAG_LAST | rd.FLAG_PF | rd.FLAG_DROP, seg=9, rep=8,
                           valid=100, lost=7, ddr_kib=1234)
        d = rd.decode_block(b)
        self.assertEqual(d["errors"], [])
        self.assertEqual((d["pos"], d["seq"], d["lost"], d["sid"], d["ts"], d["seg"], d["rep"],
                          d["valid"], d["ddr_kib"]),
                         (0x123456789, 0x123456789 + 7, 7, 0xDEADBEEF, 99, 9, 8, 100, 1234))
        self.assertEqual(d["pos_lo"], 0x23456789)
        self.assertEqual(rd.flags_str(d["flags"]), "LAST|PF|DROP")

    def test_negative_controls_each_corruption_detected(self):
        good = rd.build_block(2, sid=1)
        self.assertEqual(rd.decode_block(good)["errors"], [])

        def flip(off, bit=0):
            b = bytearray(good)
            b[off] ^= 1 << bit
            return bytes(b)
        cases = {
            "magic": (flip(0), "magic"),
            "magic_end": (flip(4092), "magic_end"),
            "payload bit": (flip(1000), "payload"),
            "crc field": (flip(4088), "crc"),
            "header bit": (flip(25), "crc"),            # ts byte: only CRC sees it
            "pos_lo": (flip(4084), "pos_lo"),
            "seq_lo": (flip(4080), "seq_lo"),
            "ver": (flip(32), "ver"),
            "valid": (flip(39, 7), "valid"),
        }
        for name, (blk, want) in cases.items():
            self.assertIn(want, rd.decode_block(blk)["errors"], name)
        # seq - lost != pos
        self.assertIn("seq_lost_pos", rd.decode_block(rd.build_block(2, seq=9))["errors"])

    def test_bad_length(self):
        with self.assertRaises(rd.BlockError):
            rd.decode_block(b"\0" * 4095)

    def test_chain_check(self):
        blks = [rd.build_block(i, sid=7, flags=rd.FLAG_LAST if i == 2 else 0) for i in range(3)]
        dec = [rd.decode_block(b) for b in blks]
        self.assertEqual(rd.chain_check(dec, sid=7), [])
        self.assertTrue(rd.chain_check([dec[0], dec[2]], sid=7))          # hole
        self.assertTrue(rd.chain_check(dec[:2], sid=7))                   # no LAST
        self.assertTrue(rd.chain_check(dec, sid=8))                       # wrong sid

    def test_hex_text_roundtrip(self):
        blk = rd.build_block(1, sid=5)
        text = "// c\n" + "\n".join(blk[i:i + 16][::-1].hex() for i in range(0, 4096, 16)) + "\n"
        self.assertEqual(rd.parse_hex_text(text), blk)
        with self.assertRaises(rd.BlockError):
            rd.parse_hex_text("0123\n01234567\n")                          # mixed widths

    def test_read_blocks_file_formats(self):
        blk = rd.build_block(0, sid=5)
        with tempfile.TemporaryDirectory() as td:
            pb, ph = os.path.join(td, "b.bin"), os.path.join(td, "b.hex")
            with open(pb, "wb") as f:
                f.write(blk * 2)
            with open(ph, "w") as f:
                f.write("\n".join(blk[i:i + 16][::-1].hex() for i in range(0, 4096, 16)))
            self.assertEqual(rd.read_blocks(pb), [blk, blk])
            self.assertEqual(rd.read_blocks(ph), [blk])


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""check_pattern.py - verify the test pattern recorded by rtl/board/kc705_top.sv.

The on-chip generator writes, starting at LBA 0, 128-bit beats (16 bytes, little-endian words):

    beat n (n = 0, 1, 2, ..., 32 bit counter, wraps after 2**32 beats = 64 GiB):
        word0 = n
        word1 = n * 7           (mod 2**32)
        word2 = n ^ 0xA5A5A5A5
        word3 = ~n              (32 bit)

i.e. the AXI-S beat { ~n, n ^ 32'hA5A5A5A5, n * 7, n } of sim/tb/tb_top.sv, byte 0 = LSB of n.

Usage
    python check_pattern.py IMAGE_OR_DEVICE [--bytes N] [--offset N] [--start-lba N --lba-size 512]
                                            [--chunk-mib 16] [--expect N]

    IMAGE_OR_DEVICE   a raw image made with dd, or a block device (Linux /dev/nvme1n1, Windows \\\\.\\PhysicalDrive2;
                      raw devices need Administrator / root, reads are done in multiples of 4096 bytes)
    --bytes N         verify at most N bytes (default: to the end of the file / device).  Accepts K/M/G/T suffixes (1024 based)
    --offset N        byte offset of beat 0 inside IMAGE (default 0 = LBA 0)
    --start-lba N     alternative to --offset: LBA of beat 0 (with --lba-size)
    --expect N        exit status 1 unless at least N bytes verified (suffixes allowed), e.g. --expect 1G for a 1 GiB run

Reports the number of verified bytes (the contiguous prefix that matches) and the first mismatch
(byte offset, LBA, beat number, expected / found 32-bit word).  Exit status 0 = everything read matched
(and --expect satisfied), 1 = mismatch or too short, 2 = usage / I/O error.

Typical use:
    dd if=/dev/nvme1n1 of=rec.img bs=1M count=1024 iflag=direct     # optional: copy first
    python check_pattern.py rec.img --expect 1G
    python check_pattern.py /dev/nvme1n1 --bytes 8G                  # or directly from the device

Note: a run stopped with the STOP button in "unlimited" mode ends with the last, partially filled ring slot not
written; the checker then reports the first mismatch at the end of the recording (old data of the SSD), and the
"verified" count is the size of the recording.
"""
import argparse
import sys
import time

try:
    import numpy as np
except ImportError:  # slow pure-python fallback
    np = None

BEAT = 16
MASK = 0xFFFFFFFF


def parse_size(txt):
    txt = txt.strip().upper()
    mult = {"K": 1 << 10, "M": 1 << 20, "G": 1 << 30, "T": 1 << 40}
    if txt and txt[-1] in mult:
        return int(float(txt[:-1]) * mult[txt[-1]])
    return int(txt, 0)


def expected_words_np(first_beat, nbeats):
    n = (np.arange(nbeats, dtype=np.uint64) + np.uint64(first_beat)) & np.uint64(MASK)
    n = n.astype(np.uint32)
    out = np.empty((nbeats, 4), dtype="<u4")
    out[:, 0] = n
    out[:, 1] = n * np.uint32(7)                   # wraps modulo 2**32
    out[:, 2] = n ^ np.uint32(0xA5A5A5A5)
    out[:, 3] = ~n
    return out


def expected_bytes(first_beat, nbeats):
    if np is not None:
        return expected_words_np(first_beat, nbeats).tobytes()
    import struct
    parts = []
    for i in range(nbeats):
        n = (first_beat + i) & MASK
        parts.append(struct.pack("<4I", n, (n * 7) & MASK, n ^ 0xA5A5A5A5, (~n) & MASK))
    return b"".join(parts)


def first_diff(a, b):
    """index of the first differing byte of two equal-length bytes objects, or -1"""
    if a == b:
        return -1
    if np is not None:
        d = np.nonzero(np.frombuffer(a, np.uint8) != np.frombuffer(b, np.uint8))[0]
        return int(d[0])
    for i, (x, y) in enumerate(zip(a, b)):
        if x != y:
            return i
    return -1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image")
    ap.add_argument("--bytes", type=parse_size, default=None)
    ap.add_argument("--offset", type=parse_size, default=None)
    ap.add_argument("--start-lba", type=parse_size, default=None)
    ap.add_argument("--lba-size", type=parse_size, default=512)
    ap.add_argument("--chunk-mib", type=int, default=16)
    ap.add_argument("--expect", type=parse_size, default=None)
    a = ap.parse_args()

    offset = a.offset if a.offset is not None else (a.start_lba or 0) * a.lba_size
    chunk = max(1, a.chunk_mib) * (1 << 20)
    chunk -= chunk % 4096
    limit = a.bytes

    try:
        f = open(a.image, "rb", buffering=0)
        f.seek(offset)
    except OSError as e:
        print("error: cannot open %s: %s" % (a.image, e), file=sys.stderr)
        return 2

    verified = 0
    mismatch = None
    t0 = time.time()
    last_print = t0
    with f:
        while limit is None or verified < limit:
            want = chunk if limit is None else min(chunk, limit - verified)
            want_rd = (want + 4095) // 4096 * 4096          # raw devices: whole sectors
            try:
                data = f.read(want_rd)
            except OSError as e:
                print("error: read at byte %d failed: %s" % (offset + verified, e), file=sys.stderr)
                return 2
            if not data:
                break
            data = data[:want]
            nfull = len(data) // BEAT * BEAT
            tail = len(data) - nfull
            first_beat = (verified // BEAT) & MASK
            exp = expected_bytes(first_beat, (len(data) + BEAT - 1) // BEAT)[:len(data)]
            d = first_diff(data, exp)
            if d >= 0:
                verified += d
                pos = offset + verified
                beat = (verified // BEAT)
                word = (verified % BEAT) // 4
                got = int.from_bytes(data[d - d % 4:d - d % 4 + 4].ljust(4, b"\0"), "little")
                exw = int.from_bytes(exp[d - d % 4:d - d % 4 + 4].ljust(4, b"\0"), "little")
                mismatch = (pos, pos // a.lba_size, beat, word, exw, got)
                break
            verified += len(data)
            if tail:                       # file ends in the middle of a beat
                break
            if len(data) < want:           # EOF
                break
            now = time.time()
            if now - last_print > 5:
                print("  ... %.2f GiB verified (%.0f MB/s)" % (verified / 2**30, verified / 1e6 / (now - t0)), file=sys.stderr)
                last_print = now

    dt = max(time.time() - t0, 1e-9)
    print("verified bytes : %d (%.3f GiB, %d beats of 16 B) in %.1f s" % (verified, verified / 2**30, verified // BEAT, dt))
    ok = True
    if mismatch:
        pos, lba, beat, word, exw, got = mismatch
        print("first mismatch : byte offset %d (LBA %d + %d, beat %d, word %d): expected 0x%08X, found 0x%08X"
              % (pos, lba, pos - lba * a.lba_size, beat & MASK, word, exw, got))
        print("RESULT         : MISMATCH")
        ok = False
    else:
        print("first mismatch : none within the %s" % ("requested range" if limit is not None else "image"))
        print("RESULT         : OK")
    if a.expect is not None and verified < a.expect:
        print("expected at least %d verified bytes, got %d" % (a.expect, verified))
        ok = False
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())

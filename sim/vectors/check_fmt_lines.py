#!/usr/bin/env python3
"""Feed the lines written by tb_rec_tx_fmt (sim/work/rec_tx_fmt/lines_dump.txt) through
tools/recorder.py parse_line(strict=True).  Usage: python sim/vectors/check_fmt_lines.py [file]"""
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import recorder  # noqa: E402

path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, "sim", "work", "rec_tx_fmt", "lines_dump.txt")
n = bad = 0
kinds = {}
for ln in open(path):
    ln = ln.rstrip("\n")
    n += 1
    try:
        m = recorder.parse_line(ln, strict=True)
        k = (m.kind, m.name)
        kinds[k] = kinds.get(k, 0) + 1
    except recorder.ParseError as e:
        bad += 1
        print("PARSE ERROR:", e)
for k, v in sorted(kinds.items()):
    print("%-8s %-8s %d" % (k[0], k[1], v))
print("%d lines, %d rejected, longest %d chars" % (n, bad, max(len(l.rstrip("\n")) for l in open(path))))
sys.exit(1 if bad else 0)

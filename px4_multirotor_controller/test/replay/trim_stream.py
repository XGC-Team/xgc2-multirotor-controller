#!/usr/bin/env python3
"""Cut a PMCRPLY1 replay stream to the records within SECONDS of its first record.

Usage: trim_stream.py IN.stream OUT.stream SECONDS
"""
import struct
import sys

src, dst, seconds = sys.argv[1], sys.argv[2], float(sys.argv[3])
data = open(src, "rb").read()
assert data[:8] == b"PMCRPLY1"
out = bytearray(b"PMCRPLY1")
offset, first, kept = 8, None, 0
while offset < len(data):
    t, kind, length = struct.unpack_from("<QBI", data, offset)
    end = offset + 13 + length
    first = t if first is None else first
    if (t - first) * 1e-9 > seconds:
        break
    out += data[offset:end]
    kept += 1
    offset = end
open(dst, "wb").write(out)
print(f"{kept} records, {len(out)} bytes -> {dst}")

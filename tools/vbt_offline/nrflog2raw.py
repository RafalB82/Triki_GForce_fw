#!/usr/bin/env python3
"""nrflog2raw.py — parser loga nRF Connect (wire v1 14B) -> surowe raw12 na stdout.

Replay realnych danych przez FW VBT offline (plan VBT test C):
  python3 nrflog2raw.py "Log 2026-08-30 20_37_17.txt" | ./vbt_offline stdin > velocity.csv
Kolumny wyjscia: frame;v_new_mm_s;v_old_mm_s;flags
"""
import re
import sys

def main():
    if len(sys.argv) != 2:
        print("uzycie: nrflog2raw.py <log nrfconnect .txt>", file=sys.stderr)
        return 1
    n = skipped = 0
    out = sys.stdout.buffer
    for line in open(sys.argv[1], encoding="utf-8", errors="replace"):
        if "Notification received" not in line:
            continue
        m = re.search(r"value: \(0x\) ((?:[0-9A-Fa-f]{2}-)+[0-9A-Fa-f]{2})", line)
        if not m:
            continue
        b = bytes(int(x, 16) for x in m.group(1).split("-"))
        if len(b) != 14 or b[0] != 0x22 or b[1] != 0x00:
            skipped += 1
            continue
        out.write(b[2:])          # raw12: gyro6+acc6 i16LE
        n += 1
    print(f"ramek: {n}, pominieto: {skipped} (nie wire v1 14B)", file=sys.stderr)
    return 0

if __name__ == "__main__":
    sys.exit(main())

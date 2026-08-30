#!/usr/bin/env bash
# Harness offline VBT (C6) — host cc, kompiluje triki/trikig_vbt.c 1:1 z FW.
set -euo pipefail
cd "$(dirname "$0")"
cc -O2 -Wall -Wextra -I../../triki ../../triki/trikig_vbt.c main.c -lm -o vbt_offline
echo "OK: ./vbt_offline [rest60|rot|rot_move|rep|stdin|all]"

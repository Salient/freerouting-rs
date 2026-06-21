#!/usr/bin/env bash
# WSL-side round-trip driver. Routes an Altium-exported DSN through freerouting-rs and
# stages the .rte where Altium can import it.
#   usage: harness/roundtrip.sh <board-name>   (expects <work>/<board>.dsn to exist)
set -euo pipefail
BOARD="${1:?usage: roundtrip.sh <board-name>}"
WORK="/mnt/c/Users/jheller2/altium_rte_test"
DSN="$WORK/$BOARD.dsn"
RTE="$WORK/$BOARD.rte"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

[ -f "$DSN" ] || { echo "ERROR: $DSN not found (run ExportDsn in Altium first)"; exit 1; }
echo "routing $DSN -> $RTE"
cargo run -q --manifest-path "$ROOT/Cargo.toml" --bin freerouting-rs -- route "$DSN" -o "$RTE" "${@:2}"
echo "staged $RTE  ($(wc -l < "$RTE") lines). Now run ImportRte in Altium."

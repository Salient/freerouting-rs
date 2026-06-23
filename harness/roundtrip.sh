#!/usr/bin/env bash
# WSL-side round-trip driver. Routes an Altium-exported DSN through freerouting-rs and
# stages the .rte where Altium can import it.
#   usage: harness/roundtrip.sh <board-name> [extra route args...]
#   expects <WORK>/<board>.dsn to exist (from the Altium ExportDsn procedure).
#
# WORK defaults to the FREEROUTING_WORK env var, else the Windows shared folder under the
# current user's home — machine-agnostic (was hardcoded to a specific user).
set -euo pipefail
BOARD="${1:?usage: roundtrip.sh <board-name> [route args...]}"
: "${FREEROUTING_WORK:=/mnt/c/Users/$USER/altium_rte_test}"
WORK="$FREEROUTING_WORK"
DSN="$WORK/$BOARD.dsn"
RTE="$WORK/$BOARD.rte"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

[ -f "$DSN" ] || { echo "ERROR: $DSN not found (run ExportDsn in Altium first; set FREEROUTING_WORK if needed)"; exit 1; }
echo "routing $DSN -> $RTE"
cargo run -q --manifest-path "$ROOT/Cargo.toml" --bin freerouting-rs -- route "$DSN" -o "$RTE" "${@:2}"
echo "staged $RTE  ($(wc -l < "$RTE") lines). Now run ImportRte (or RoundTrip) in Altium."

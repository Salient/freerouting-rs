# Altium round-trip test harness

Goal: a fast loop to validate freerouting-rs against real Altium —
**create test board in Altium → export DSN → route in freerouting-rs → import RTE back → verify**.

## The pieces

- `altium_roundtrip.pas` — Altium DelphiScript with two procedures:
  - `ExportDsn`   : exports the current PCB to `<work>\<board>.dsn` (Specctra design).
  - `ImportRte`   : imports `<work>\<board>.rte` onto the current PCB, then writes a
    `<work>\import_result.txt` with track/via counts so the harness can verify.
  - `ReportCounts`: read-only track/via/component count (sanity probe).
- `roundtrip.sh` — WSL driver: invokes freerouting-rs on the exported DSN and stages
  the `.rte` where Altium can import it. Altium-side export/import is triggered via the
  GUI (see "Triggering" below) because `-RScript` only fires on a cold Altium start.

## Conventions learned the hard way (Altium interop)

- All files Altium reads/writes must be **ASCII + CRLF**. The harness normalizes.
- Route files (`.rte`) MUST be top-level `(routes ...)`, **CRLF**, with each wire/via on
  ONE line carrying its own `(net ...)` and `(type ...)`. (freerouting-rs emits this; see
  freerouting-rs-spec/ALTIUM_COMPAT.md.)
- `-RScript "file|Proc"` is **ignored when Altium is already running**; it only auto-runs
  on a cold launch. With a live session, trigger scripts from the GUI
  (File > Run Script, or open the .pas and press F9) — one click per step.
- Work dir on the Windows side: `C:\Users\jheller2\altium_rte_test\`
  (WSL path `/mnt/c/Users/jheller2/altium_rte_test/`).

## Triggering (current, semi-automated)

1. In Altium, open the test board.
2. Run `ExportDsn` (writes `<board>.dsn` to the work dir).
3. On the WSL side: `harness/roundtrip.sh <board>` — routes the DSN, stages `<board>.rte`.
4. In Altium, run `ImportRte`; check the board + `import_result.txt`.

## Test boards

`make_test_boards/` will hold notes/specs for the simple feature-coverage boards
(varied clearances, trace widths, via styles, polygon pours) to build in Altium. Each
exercises one feature so a failed round-trip points at a specific cause.

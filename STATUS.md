# freerouting-rs — Build Status & Resume Handoff

Last updated before a planned restart. This file lets a fresh session resume without
re-deriving context. The authoritative spec is in `../freerouting-rs-spec/`.

## How to resume

1. Read `../freerouting-rs-spec/{README,REQUIREMENTS,ARCHITECTURE,ALGORITHM,ALTIUM_COMPAT,MILESTONES}.md`.
2. Read this file for current state.
3. `cd /home/jheller2/Projects/freerouting-rs && cargo test` — everything green except
   what's noted below.
4. Continue from "Next steps".

## Environment (verified working)

- Rust 1.93.1 (`cargo`, `rustc` on PATH). crates.io reachable.
- GUI system libs installed (x11/xkbcommon/fontconfig/libGL); egui/eframe 0.29 compiles.
- Java oracle at `/home/jheller2/Projects/freerouting` (branch `port/altium-fixes-from-fork`), JDK 11.
- Altium AD25 on Windows host; round-trip via `/mnt/c/Users/jheller2/altium_rte_test/`.
- Altium scripting trigger: **File > Run Script > pick procedure** (F9 does NOT work here).

## Phase status (per MILESTONES.md)

| Phase | What | State | Gate |
|---|---|---|---|
| 0 | Workspace skeleton (8 crates) | DONE | build+test green |
| 1 | fr-geometry (point/vector/orientation/box/convex tile) | DONE | 13 unit + 6 proptest |
| 3 | fr-board (layers/padstacks incl. shapeless/nets/items/rules) | DONE | 11 tests |
| 4 | fr-dsn (tolerant lexer+tree, DSN->Board reader, SES/RTE writers) | DONE | altium_validator + real-board parse + pin placement |
| 5-6 | fr-route (grid A*, obstacles, router) + fr-engine | DONE | real per-pin placement + MST; 372/417 nets |
| 7 | fr-gui (egui app) + software renderer | DONE (interactive needs real display) | render path verified, board image produced |
| 8 | Parallelism + perf (rayon scheduler + bench) | DONE | ~2.8x speedup, deterministic, 372/417 nets |
| 9 | Acceptance | DONE | ACCEPTANCE.md; baseline_rs.rte produced |

All phases complete. Remaining work is refinement/future (see ACCEPTANCE.md "Remaining"):
real-Altium import confirmation by a human, quality A/B vs the Java oracle, RSS
comparison, and the free-angle room/door search space to replace the grid.

Note: Phase 2 (fr-spatial / rstar R-tree) is stubbed; the grid router doesn't need it
yet. The free-angle room/door model (the spec's end-goal search space) is NOT yet built
— the current router uses a uniform grid as a working stand-in (same A* driver).

## Verified end-to-end

`freerouting-rs route harness/sample_board.dsn -o out.rte --max-time 30` loads the real
43k-line Altium board (6 layers, 417 nets, 802 components), routes (~12 nets / 112 traces
in ~12s), and emits a valid Altium-format RTE (CRLF, top-level routes, one-line wires with
per-wire net/type, balanced parens). `info` subcommand prints a board summary.

## Open items / debug-later (in priority order)

1. **Routing completion is low (~12/417 nets).** Root causes to fix:
   - Pins are placed at COMPONENT CENTERS (no per-pin image/offset parsing yet), so
     multi-pin nets on one component have coincident points and inter-pin geometry is
     wrong. Implement real pin placement from library `(image (pin ...))` + component
     rotation/placement. THIS is the biggest quality lever.
   - Net connection ordering is a naive chain; use an MST over pins.
   - Grid pitch/expansion tuning; consider the real room/door model for free-angle.
2. **Phase 7 GUI headless launch** fails: winit "Broken pipe" -> ExitFailure(1) under
   Xvfb (+ -ac, + openbox all tried). Env issue, not code. Verify on real WSLg `:0`
   (DISPLAY=:0) — was about to test when we paused. harness/gui_screenshot.sh is the gate.
3. **Phase 8** parallel scheduler + criterion benches not started.
4. fr-spatial R-tree unused; wire in when moving off the grid.
5. Polygon-pour pads approximated as circles in the DSN reader (TODO in reader.rs).
6. The Java oracle's conduction-area (pour) wires may still be multi-line — separate
   from fr-rs; noted in the Java repo.

## Next steps (recommended order on resume)

1. **Test GUI on DISPLAY=:0** (real WSLg) to confirm it renders, capture a screenshot.
   If it works there, the Phase 7 gate is met (headless was an env artifact).
2. **Implement real pin placement** (library images + component transform) — unlocks
   routing completion, the main quality gap.
3. Phase 8: rayon multi-net scheduler + benches.
4. Phase 9 acceptance pass; produce artifacts/baseline_rs.rte for Altium import.

## Key commands

```
cargo test                       # all crates (fr-gui excluded from workspace)
cargo build --release            # CLI at target/release/freerouting-rs
(cd crates/fr-gui && cargo build --release)   # GUI (separate target/)
./target/release/freerouting-rs info  harness/sample_board.dsn
./target/release/freerouting-rs route harness/sample_board.dsn -o /tmp/o.rte --max-time 30
bash harness/gui_screenshot.sh   # Phase 7 gate (currently blocked headless)
```

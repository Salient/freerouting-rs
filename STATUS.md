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

All phases complete. Post-acceptance correctness/quality work done:
- Fixed trace SHORTING (incremental routing + width/clearance stamping; DRC gate ==0).
- Fixed via padstack selection (uses the board's real routing via, not an arbitrary pad).
- Routing quality: local-first net ordering + multi-pass retry -> 374/417 nets (90%),
  934 traces, 49 vias, 0 shorts on the real board, ~3.5s.
- GUI runs on WSLg via the eframe wgpu backend; Open via typed path (native dialog
  unavailable under WSLg).

GUI suite (task #8) DONE: in-app file browser, routing-parameter config panel
(time/threads/width/clearance/layers), manual commands (Route/Clear/Fit), ratsnest +
incompletes readout, net highlight, layer legend/toggles. Runs on WSLg via wgpu.

Honest DRC status: rebuilt the DRC to use true copper geometry (trace WIDTH + pad
radius), exposing that the real shorts are trace-to-PAD, not trace-to-trace
(trace-to-trace == 0). ~14 trace-to-pad shorts remain on the dense real board because
component pads are larger than the grid pitch - a fundamental grid-routing limit.

Remaining / future:
- **Free-angle room/door search model (task #9)** - now the REQUIRED next step: it
  represents exact pad/trace geometry and eliminates the trace-to-pad shorts the grid
  router cannot avoid. Also lifts completion past ~76% and gives any-angle/shorter
  traces. This is the path to electrically-clean output.
- Human real-Altium import confirmation; quality A/B vs Java oracle; RSS comparison.

Note: Phase 2 (fr-spatial / rstar R-tree) is stubbed; the grid router doesn't need it
yet. The free-angle room/door model (the spec's end-goal search space) is NOT yet built
— the current router uses a uniform grid as a working stand-in (same A* driver).

## Verified end-to-end (current)

`freerouting-rs route harness/sample_board.dsn -o out.rte` loads the real 43k-line
Altium board (6 layers, 417 nets, 802 components), routes **374/417 nets, 934 traces,
49 vias in ~3.5s** (incremental + MST + local-first order + multi-pass), and emits a
valid Altium-format RTE (CRLF, top-level routes, one-line wires w/ per-wire net/type,
scaled-int coords, all inside the outline). The egui GUI runs on WSLg `:0` (wgpu
backend) with file browser, config panel, ratsnest, net highlight, layer legend,
Route/Clear/Fit/Export.

## THE open problem (start here on resume)

**Trace-to-PAD shorts (~14 on the real board).** Verified by the true copper-geometry
DRC (fr-engine: `drc_short_count` = trace-trace = 0; `drc_trace_pin_short_count` ~14).
Cause: component pads (13-16 mil radius) are LARGER than the routing-grid pitch, so a
trace routed to the cell next to a pad still overlaps its copper. Finer grids trade
this for worse completion (tested). This is a fundamental grid-routing limit.

**=> Implement the free-angle room/door search model (task #9).** It represents exact
pad/trace geometry with true clearance and structurally eliminates trace-to-pad shorts
(this is why real freerouting uses it, not a grid). It also lifts completion past ~76%
and yields any-angle/shorter traces. This is the required next step for usable output.
The A* driver (fr-route/astar.rs) and engine API are structured so only the neighbour
generation / search space changes; keep the DRC gates (tests/drc.rs, tests/offboard.rs)
green and tighten trace-pin to 0 once done.

## Smaller open items
- fr-spatial (rstar) R-tree is still unused; wire it in for the room/door obstacle queries.
- Polygon-pour pads approximated as circles in the DSN reader (reader.rs TODO).
- GUI: live redraw DURING routing (currently routes synchronously then redraws); a
  background routing thread + progress channel would let the canvas animate.
- Human real-Altium import confirmation; quality A/B vs Java oracle; RSS comparison.

## Next steps (recommended order on resume)

1. **Build the free-angle room/door router (task #9)** — the required fix for
   trace-to-pad shorts and higher completion. Biggest, highest-value work item.
2. Wire fr-spatial R-tree into obstacle queries as part of #9.
3. Re-run DRC gates; tighten trace-pin shorts gate toward 0.
4. GUI live-routing thread + progress.

## Key commands

```
cargo test                       # all crates (fr-gui excluded from workspace)
cargo build --release            # CLI at target/release/freerouting-rs
(cd crates/fr-gui && cargo build --release)   # GUI (separate target/)
./target/release/freerouting-rs info  harness/sample_board.dsn
./target/release/freerouting-rs route harness/sample_board.dsn -o /tmp/o.rte --max-time 30
bash harness/gui_screenshot.sh   # Phase 7 gate (currently blocked headless)
```

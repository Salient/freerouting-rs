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
- Routing quality: local-first net ordering + multi-pass retry -> 316/417 nets,
  594 traces, 29 vias, 0 trace-trace AND 0 trace-pad shorts on the real board, ~1.4s
  release (deterministic, byte-identical output across thread counts; verified
  2026-06-22).
- **Trace-to-PAD shorts SOLVED (was the open problem):** an exact-geometry obstacle
  index (fr-spatial `ObstacleIndex`, rstar R-tree, pads/vias as discs + traces as fat
  segments, per layer) now backs an A* edge validator (fr-route `EdgeValidator`). Each
  in-plane A* move is gated by an exact segment-to-copper clearance test, so a trace can
  no longer clip a different-net pad between two passable grid cells. DRC trace-pin gate
  tightened from "<60" to "==0".
- GUI runs on WSLg via the eframe wgpu backend; Open via typed path (native dialog
  unavailable under WSLg).

GUI suite (task #8) DONE: in-app file browser, routing-parameter config panel
(time/threads/width/clearance/layers), manual commands (Route/Clear/Fit), ratsnest +
incompletes readout, net highlight, layer legend/toggles. Runs on WSLg via wgpu.

Honest DRC status: the DRC uses true copper geometry (trace WIDTH + pad radius).
trace-to-trace == 0 (incremental stamping) and trace-to-PAD == 0 (exact edge validator,
above). The output is now electrically clean on the real board.

Free-angle room/door model (DONE stages 1-7; see crates/fr-route/ROOMDOOR_DESIGN.md):
faithful staged port of the Java autoroute model, each stage committed + tested.
- 1 free-space rooms (restrain_shape port), 2 doors, 3 maze A*, 4 any-angle backtrace,
  5 angle restriction (any/45/90), 6 vias/multi-layer, 7 push/shove (rip-up & reroute).
- fr-route: room.rs / maze.rs / locate.rs; single-connection router
  `route_connection_roomdoor` (RoomDoorOptions). fr-engine: `interactive::InteractiveRouter`
  (begin/preview/commit/commit_shove) for manual routing.
- Proven on the real board (tests/roomdoor_ab.rs): sampled two-pin same-layer nets route
  any-angle, ALL DRC-clean, total length near the straight-line lower bound.
- The GRID router is still the DEFAULT batch router (`route_board`, electrically clean via
  the exact validator). The room/door model powers INTERACTIVE routing in the GUI; making
  it the default batch router (rip-up-reroute over all nets) is the remaining quality step.

GUI features:
- Tier 1: real per-layer pad shapes, filled board outline with contrast, trace/pad/via
  selection + hover info + selection panel. (padgeom.rs / picking.rs.)
- Manual routing (room/door model): "Manual route" panel — Route mode, snap angle
  (Any/45/90), Allow-vias, Shove (rip-up & reroute), active layer. Click to draw, live
  green/red preview, right-click/Esc to finish. (fr_engine::interactive.)

- Make the room/door model the default batch router; human real-Altium import
  confirmation; quality A/B vs Java oracle; RSS comparison.

Note: Phase 2 (fr-spatial / rstar R-tree) is now USED: `ObstacleIndex` backs both the
grid A* exact edge validator AND the room/door model's obstacle queries.

## Verified end-to-end (current)

`freerouting-rs route harness/sample_board.dsn -o out.rte` loads the real 43k-line
Altium board (6 layers, 417 nets, 802 components), routes **316/417 nets, 594 traces,
29 vias in ~1.4s release** (incremental + MST + local-first order + multi-pass + exact
edge validator; 0 trace-trace and 0 trace-pad shorts), and emits a
valid Altium-format RTE (CRLF, top-level routes, one-line wires w/ per-wire net/type,
scaled-int coords, all inside the outline). The egui GUI runs on WSLg `:0` (wgpu
backend) with file browser, config panel, ratsnest, net highlight, layer legend,
Route/Clear/Fit/Export.

## THE open problem — SOLVED (2026-06-22)

**Trace-to-PAD shorts: was ~14, now 0.** Root cause was that the grid A* checked
passability only at node CENTERS, so a trace segment (esp. a diagonal) could run between
two passable cells yet clip a pad larger than the grid pitch. Fix: exact-geometry edge
validation.
- `fr-spatial::ObstacleIndex` — per-layer rstar R-tree of true copper (pads/vias as
  discs, traces as fat segments), tagged by net. `segment_is_clear(layer,a,b,half,net,
  clearance)` answers the exact copper-to-copper clearance question.
- `fr-route::EdgeValidator` — passed into `astar::search`; each in-plane move is rejected
  unless its swept trace segment clears all different-net copper by `clearance`.
- `fr-engine` builds the index from board pads/vias, passes the validator to every
  `route_connection`, and stamps each committed trace/via into BOTH the grid ObstacleMap
  and the exact index, so later nets see prior geometry exactly.
- DRC gate tightened: `drc_trace_pin_short_count` now asserted `== 0` (was `< 60`).

## Next biggest lever (quality, not correctness)
**Free-angle room/door search model (task #9)** — no longer required for electrical
cleanliness (achieved above), but still the path to higher completion (>76%) and
any-angle/shorter traces. The A* driver and `EdgeValidator` are now structured so the
room/door model changes only the neighbour generation; the exact clearance kernel
(fr-spatial) is already the obstacle query it needs.

## Outstanding work — see OUTSTANDING.md (triage scan 2026-06-22)
Top HIGH-impact correctness gaps found by the scan:
- **Keepouts ignored** by the DSN reader (175+ regions) — router can cross them.
- **Pre-existing `(wiring)` discarded** (5870 wires) — we re-route from scratch and don't
  avoid existing copper.
Plus: grid is still the default batch router (room/door is built but interactive-only);
per-net class/rule widths ignored; A/B + RSS + live-Altium import still manual.

## Smaller open items
- GUI: live redraw DURING routing (synchronous then redraw); a background routing thread +
  progress channel would animate the canvas.
- Polygon pad clearance halo is approximate in the GUI; convex pads use a circumscribed
  disc in the obstacle index/DRC (conservative).

## Next steps (recommended order on resume)

Correctness is done (0 shorts of either kind). Remaining work is QUALITY/UX:
1. **Lift completion past 76%** — full rip-up-and-reroute, or the free-angle room/door
   router (task #9, reuses `ObstacleIndex`). Biggest remaining lever.
2. Performance: the exact validator does an R-tree query per A* edge (~1.4s release, fine
   now). If the room/door work makes searches longer, cache/restrict queries to congested
   regions.
3. GUI live-routing thread + progress.
4. Human real-Altium import confirmation; quality A/B vs Java oracle; RSS comparison.

## Key commands

```
cargo test                       # all crates (fr-gui excluded from workspace)
cargo build --release            # CLI at target/release/freerouting-rs
(cd crates/fr-gui && cargo build --release)   # GUI (separate target/)
./target/release/freerouting-rs info  harness/sample_board.dsn
./target/release/freerouting-rs route harness/sample_board.dsn -o /tmp/o.rte --max-time 30
bash harness/gui_screenshot.sh   # Phase 7 gate (currently blocked headless)
```

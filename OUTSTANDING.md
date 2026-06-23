# Outstanding work — triage scan (2026-06-22)

Scan of TODOs, unparsed DSN constructs, planning-doc remainders, and acceptance gaps,
after the GUI/engine feedback round. Ordered by impact. No `todo!()`/`unimplemented!()`
panics exist in the code; the items below are missing features / approximations.

## Correctness gaps (router can produce wrong output)

1. **Keepouts ignored.** The DSN reader does not parse `(keepout ...)` (175 + 2 on the
   real board) — routing-forbidden regions. The router can route through them. No keepout
   field exists on the board model. → add Keepout to fr-board, parse in reader, stamp into
   ObstacleMap + ObstacleIndex (as NO_NET obstacles). HIGH impact.

2. **Pre-existing `wiring` discarded.** The DSN `(wiring ...)` has 5870 pre-routed wires +
   vias (fixed/protected copper). The reader ignores the whole section, so we re-route
   from scratch and don't route around existing copper. → parse wiring into fixed
   traces/vias; treat as obstacles; don't rip them up. HIGH impact for real boards.

3. **`(plane ...)` / power-plane layers ignored.** Copper-pour planes (net-tied) aren't
   parsed; traces could overlap a plane of another net. MEDIUM (board has planes via the
   power layers). 

## Quality / completeness (works, but approximate or limited)

4. **Grid is still the default BATCH router.** Room/door model (stages 1-7) is built and
   powers interactive routing, but `route_board` is the uniform-grid A*. Making room/door
   the default batch router (rip-up-reroute over all nets) is the biggest quality lever
   (completion >76%, any-angle). (STATUS, ROOMDOOR_DESIGN, ACCEPTANCE all note this.)

5. **Class/rule per-net widths & clearances ignored.** The DSN `(class ...)` (8) and net
   rules set per-net width/clearance; we use one global default_width/clearance. MEDIUM.

6. **Polygon pad clearance halo is approximate** in the GUI (draws the pad outline, not a
   true offset polygon). Circles/traces are exact. LOW (cosmetic).

7. **Convex pads use a circumscribed disc in the obstacle index/DRC** (rotation-invariant,
   conservative — never under-reserves, slightly over-reserves at corners). Exact polygon
   clearance would recover a little routing space. LOW.

## UX / infra

8. **GUI live redraw during routing** — routing is synchronous then redraws; a background
   thread + progress channel would animate the canvas. LOW.
9. **No headless screenshot here** (Xvfb/ImageMagick absent); the software renderer +
   `--render` is the verified proxy. INFRA.

## Acceptance gaps (ACCEPTANCE.md, external/manual)

10. Human real-Altium import confirmation of the RTE. (manual)
11. Quality A/B vs the Java oracle (completion / via count / trace length). (manual)
12. RSS-vs-JVM performance comparison. (manual)

## Done in this round (for reference)
Outline (L-shape), concave fill, rect/polygon pads, pad rotation, missing-via fix,
manual-route start/net/cancel, select-highlights-net, layer cycling (↑/↓), wider tooltip,
unlimited undo/redo, Java color scheme. Grid DRC 0/0; 19 workspace test blocks green.

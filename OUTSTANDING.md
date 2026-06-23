# Outstanding work — triage scan (2026-06-22)

Scan of TODOs, unparsed DSN constructs, planning-doc remainders, and acceptance gaps,
after the GUI/engine feedback round. Ordered by impact. No `todo!()`/`unimplemented!()`
panics exist in the code; the items below are missing features / approximations.

## Correctness gaps (router can produce wrong output)

1. ~~**Keepouts ignored.**~~ FIXED (commit f3e7260): 84 keepouts parsed + enforced in both
   the grid ObstacleMap and the exact ObstacleIndex.

2. ~~**Pre-existing `wiring` discarded.**~~ FIXED (commit 467d745): 5862 wires + 308 vias
   loaded as fixed copper; displayed + treated as obstacles.

3. ~~**`(plane ...)` / power-plane layers ignored.**~~ N/A: the real board has ZERO
   `(plane)` scopes (verified — the earlier "planes" count was a miscount of layer `(type)`
   tokens). No plane copper to parse. A board that uses copper pours would need this.

## Quality / completeness (works, but approximate or limited)

4. **Grid is still the default BATCH router.** Room/door model (stages 1-7) is built and
   powers interactive routing, but `route_board` is the uniform-grid A*. Making room/door
   the default batch router (rip-up-reroute over all nets) is the biggest quality lever
   (completion >76%, any-angle). (STATUS, ROOMDOOR_DESIGN, ACCEPTANCE all note this.)

5. ~~**Class/rule per-net widths & clearances ignored.**~~ N/A for this board: its 8
   `(class)` scopes are net GROUPINGS with NO per-class `(rule (width)(clearance))`; the
   single global `(rule width 10 clearance 8)` is parsed + applied correctly. A future
   board with per-class rules would need per-net width plumbing through the router; not
   built (no current test case).

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
Outline (L-shape), concave fill, rect/polygon pads, per-pin pad rotation, overlap
detection, missing-via fix, via-pad clearance gate, manual-route start/net/cancel/complete,
Esc-exits-mode, Manual toolbar button, select-highlights-net, layer cycling (↑/↓ + wheel),
wider tooltip, unlimited undo/redo, Java color scheme. Keepouts parsed+enforced (84),
pre-existing wiring loaded (5862). Java parity: Incompletes/DRC/Stats/Components/Nets/Help
windows, Ratsnest/Violations toggles, Delete, cursor-coord status bar. Efficiency:
single-pass off-board mask, bbox-only keepout scan, reusable A* scratch (no 15MB/search
allocs). Altium harness: FREEROUTING_WORK env, DrcReport + RoundTrip procedures. UI: dark
modern visuals. Grid DRC 0/0; 19 workspace test blocks green.

## Still open (next session)
- Make the room/door model the DEFAULT BATCH router (rip-up-reroute over all nets) — the
  biggest remaining quality lever (interactive routing already uses it). #4 above.
- `(plane ...)` power-plane parsing (#3). Per-net class rules if a board has them (#5).
- GUI live redraw during routing (#8). Manual A/B vs Java oracle, RSS, live-Altium import.
- Exact (offset-polygon) convex-pad clearance instead of circumscribed disc (#6/#7).
- More Java parity (drag-component mode, stitch routing, clearance-matrix editor) — see
  the agent inventory; lower priority than batch room/door routing.

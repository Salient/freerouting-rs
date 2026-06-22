# Free-angle room/door routing model — design & staging

Port of the Java freerouting `autoroute` package to Rust. Faithful in spirit; staged so
each step builds + tests green and is committed independently.

## Java model (verified from /home/derp/Projects/freerouting/src_v19/.../autoroute/)

**Nodes — `ExpansionRoom`:**
- `CompleteFreeSpaceExpansionRoom`: maximal convex open-space tile on one layer; stored
  in a ShapeSearchTree; holds neighbour doors + target doors. The primary walkable tile.
- `ObstacleExpansionRoom`: keepout footprint of a board item (trace seg / via / pin);
  lets the maze pass through at a ripup/shove cost.
- `IncompleteFreeSpaceExpansionRoom`: transient; a seed `contained_shape` that must
  survive trimming; always completed into a CompleteFreeSpace room before use.

**Same-layer edges — `ExpansionDoor`:** shared boundary of two rooms. Geometry =
`first.shape ∩ second.shape`, computed on demand. dimension 1 = shared edge (a FloatLine),
dimension 2 = area overlap (corner/same-net). Long doors split into sections (≤ 10·half
width) each with its own MazeSearchElement.

**Terminal edges — `TargetItemExpansionDoor`:** room → a routable board item (net start /
dest). Terminates the wavefront.

**Cross-layer — `ExpansionDrill` / `DrillPage` / `DrillPageArray`:** a via candidate at a
Point spanning layers, one room per layer (`room_arr`); pages lazily materialise drill
sites per net.

**Maze search (`MazeSearchAlgo`):** best-first A* over a `TreeSet<MazeListElement>`
ordered by `sorting_value = expansion_value(g) + DestinationDistance(h)`. Pops min-f,
marks the door section occupied, writes backtrack pointers, expands the next room's doors
(lazily completing neighbour rooms via `SortedRoomNeighbours.calculate`). Terminates when
a destination `TargetItemExpansionDoor` is popped.

**Angle restriction** picks the room/neighbour variant:
- any-angle → `Simplex` (arbitrary convex), `SortedRoomNeighbours`
- 45° → `IntOctagon`, `Sorted45DegreeRoomNeighbours`
- 90° → `IntBox`, `SortedOrthogonalRoomNeighbours`

**Backtrace (`LocateFoundConnectionAlgo`)** follows backtrack_door links dest→start, then
synthesises the polyline:
- any-angle: tangent-visibility sweep through door left/right corners → min-bend free geo.
- 45/90: per door, project into shrunk room + insert a mandatory H/V or H/D45 elbow
  (`calculate_additional_corner`).

**Shove (`MazeShoveTraceAlgo`)**: when expanding into an obstacle room backed by a trace,
try to push it left/right (Adjustment flag) before pricing a ripup.

## Rust building blocks already present
- `fr_geometry::ConvexTile` ≈ Simplex: CCW convex polygon, `clip_halfplane`, `contains`,
  `bounding_box`. (Octagon/Box variants can be emulated as ConvexTile for now.)
- `fr_spatial::ObstacleIndex`: per-layer rstar R-tree of copper (discs + fat segments),
  `segment_is_clear`, `min_clearance_margin*`. The obstacle query the model needs.
- `fr_route::astar` driver structure (heap + g/h) — pattern to reuse for the maze.

## Staging (each: build+test green, commit) — STATUS
1. [DONE] **Free-space room geometry** (`room.rs`): faithful restrain_shape port —
   maximal convex free room around a seed, carved by clipping against obstacle edge lines
   (recursive split when an edge cuts the seed). Obstacles = copper inflated to octagons.
2. [DONE] **Doors** (`room.rs::Room::doors`): contiguous free-space sub-segments of a
   room's border (Java sorted-neighbour gap-fill equivalent).
3. [DONE] **Maze A*** (`maze.rs`): best-first over rooms/doors, door sectioning, clear-by-
   construction (every crossing edge validated by ObstacleIndex).
4. [DONE] **Backtrace** (`locate.rs::straighten`): string-pull to min-bend any-angle.
5. [DONE] **Angle restriction** (`locate.rs::straighten_angled`, AngleRestriction): any/45/
   90 elbow insertion (ninety_corner / fortyfive_corner ports). → task #9.
6. [DONE] **Vias / multi-layer** (`maze.rs::enqueue_vias`): layer-change frontier elements;
   via = stacked PathPoint. RoomDoorOptions/route_connection_roomdoor emit per-layer
   traces + vias.
7. [DONE] **Shove** (`fr_engine::interactive::commit_shove`): rip-up & reroute (the Java
   shove's fallback; verbatim MazeShoveTraceAlgo not feasible vs the unified index). #8.
8. [DONE] **Interactive API + GUI** (`fr_engine::interactive::InteractiveRouter`, GUI
   "Manual route" panel): begin/preview/commit/commit_shove; drag-route, live preview,
   snap angle, vias, shove, active layer. #8/#9.

REMAINING: make the room/door model the DEFAULT BATCH router (rip-up-reroute over all
nets) to replace the grid `route_board`. Until then the grid router is the batch default
(electrically clean via the exact validator) and room/door powers interactive routing.

Keep determinism (sorted iteration, integer geometry) and the DRC gates (0/0) green.

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

## Staging (each: build+test green, commit)
1. **Free-space room geometry** (`room.rs`): lazily build the maximal convex free room
   containing a seed point on a layer, by clipping a start box against obstacle-separating
   half-planes from `ObstacleIndex`. Tested: room contains seed, excludes obstacles, is
   convex. ← START HERE
2. **Doors + room graph** (`door.rs`): find shared-edge doors between adjacent rooms;
   lazy neighbour expansion. Tested on hand-built layouts.
3. **Maze A* over rooms/doors** (`maze.rs`): single net, single layer, any-angle; reach a
   target point. Tested: finds a path in open space, detours around an obstacle.
4. **Backtrace to polyline** (`locate.rs`): any-angle tangent sweep → trace corners.
   Integrate as an alternative single-connection router; A/B vs grid on the real board
   (expect shorter/any-angle traces, 0 shorts via the exact validator still applied).
5. **Angle restriction** (any/45/90) in room shape + backtrace. → unblocks task #9 (snap).
6. **Vias / multi-layer** (drill candidates between layer rooms).
7. **Shove** of existing traces. → unblocks task #8 (push/shove).
8. **Interactive single-trace API** + GUI wiring (drag-route, live preview, pad exit). #8/#9

Keep determinism (sorted iteration, integer geometry) and the DRC gates (0/0) green
throughout. The grid router stays as the default until the model reaches parity.

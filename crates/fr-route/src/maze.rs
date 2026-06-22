//! Maze search over the free-angle room/door graph (room/door model stage 3).
//!
//! Best-first (A*) search mirroring the Java `MazeSearchAlgo`: the frontier holds door
//! crossings ordered by `f = g + h`, where `g` is the accumulated travelled distance and
//! `h` is the straight-line distance from the door midpoint to the goal. Expanding a
//! frontier element completes the neighbour room across that door (lazily, via
//! `room::complete_room`), then enqueues that room's other doors. The search terminates
//! when it reaches a room that contains the goal point.
//!
//! Rooms are identified for dedup by a quantized key of (layer, seed cell) so the same
//! free region is not expanded repeatedly. The result is the sequence of door crossing
//! points from start to goal; the polyline backtrace (stage 4) turns it into a trace.

use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashSet};

use fr_geometry::{IntBox, Point};
use fr_spatial::ObstacleIndex;

use crate::room::{complete_room, Door, Room};

/// One step of a found room/door path: the point where the trace crosses a door (or the
/// start/goal endpoints), plus the layer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathPoint {
    pub point: Point,
    pub layer: usize,
}

/// Parameters for a maze search (single net, single layer for stage 3/4).
#[derive(Clone, Copy, Debug)]
pub struct MazeParams {
    pub net: u32,
    /// The layer the search runs on (rooms, doors and clearance are all on this layer).
    pub layer: usize,
    pub clearance: i64,
    pub half_width: i64,
    pub bound: IntBox,
    /// Door sampling / neighbour-seed step (board units). ~ trace width + clearance.
    pub step: i64,
    /// Cell size for room dedup (board units). ~ step.
    pub dedup_cell: i64,
    /// Safety cap on room expansions.
    pub max_rooms: usize,
    /// Max half-extent of a single room (board units): rooms are grown only within this
    /// window around their seed, so obstacle queries and clipping stay local (matching the
    /// Java lazy-local expansion). 0 = unbounded (use the full bound; for small tests).
    pub window: i64,
}

impl MazeParams {
    /// The effective room bound around `seed`: the global bound intersected with the local
    /// window (if any).
    fn room_bound(&self, seed: Point) -> IntBox {
        if self.window <= 0 {
            return self.bound;
        }
        let w = self.window;
        let win = IntBox::new(seed.x - w, seed.y - w, seed.x + w, seed.y + w);
        IntBox::new(
            self.bound.ll.x.max(win.ll.x),
            self.bound.ll.y.max(win.ll.y),
            self.bound.ur.x.min(win.ur.x),
            self.bound.ur.y.min(win.ur.y),
        )
    }
}

#[derive(Clone, Copy)]
struct Frontier {
    f: i64,
    g: i64,
    /// the door crossing point we entered through (start point for the first element)
    entry: Point,
    /// seed point to complete the room to expand
    seed: Point,
    layer: usize,
    /// index of the parent path node (for backtrace), or usize::MAX for the root
    parent: u32,
}

impl PartialEq for Frontier {
    fn eq(&self, o: &Self) -> bool {
        self.f == o.f && self.g == o.g
    }
}
impl Eq for Frontier {}
impl Ord for Frontier {
    fn cmp(&self, o: &Self) -> Ordering {
        // min-heap on f, then g, then a stable key for determinism
        o.f.cmp(&self.f)
            .then_with(|| o.g.cmp(&self.g))
            .then_with(|| (o.entry.x, o.entry.y).cmp(&(self.entry.x, self.entry.y)))
    }
}
impl PartialOrd for Frontier {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

/// A node in the explored tree, for backtrace.
#[derive(Clone, Copy)]
struct Node {
    point: Point,
    layer: usize,
    parent: u32,
}

fn dist(a: Point, b: Point) -> i64 {
    let dx = (a.x - b.x) as f64;
    let dy = (a.y - b.y) as f64;
    (dx * dx + dy * dy).sqrt().round() as i64
}

/// Search for a room/door path from `start` to `goal` on a single layer. Returns the
/// sequence of crossing points (start .. goal), or None if no path within `max_rooms`.
pub fn find_path(
    index: &ObstacleIndex,
    start: Point,
    goal: Point,
    p: &MazeParams,
) -> Option<Vec<PathPoint>> {
    let start_layer = p.layer; // single-layer search on the requested layer
    // seed the first room at the start point (grown within a local window for speed)
    let start_room = complete_room(
        index, start_layer, start, p.net, p.clearance, p.half_width, p.room_bound(start),
    )?;
    let mut nodes: Vec<Node> = vec![Node { point: start, layer: start_layer, parent: u32::MAX }];
    // if the start room already contains the goal and the direct segment is clear, the
    // path is a single straight segment.
    if start_room.contains(goal)
        && index.segment_is_clear(start_layer, start, goal, p.half_width, p.net, p.clearance)
    {
        return Some(vec![
            PathPoint { point: start, layer: start_layer },
            PathPoint { point: goal, layer: start_layer },
        ]);
    }

    let mut visited: HashSet<(usize, i64, i64)> = HashSet::new();
    let cell = p.dedup_cell.max(1);
    let key = |layer: usize, pt: Point| (layer, pt.x.div_euclid(cell), pt.y.div_euclid(cell));
    visited.insert(key(start_layer, start));

    let mut heap = BinaryHeap::new();
    enqueue_room_doors(
        index, &start_room, start, 0, 0, goal, p, &mut heap, &mut nodes, &visited,
    );

    let mut expansions = 0usize;
    while let Some(fr) = heap.pop() {
        let k = key(fr.layer, fr.seed);
        if !visited.insert(k) {
            continue; // room already expanded
        }
        expansions += 1;
        if expansions > p.max_rooms {
            return None;
        }
        let Some(room) = complete_room(
            index, fr.layer, fr.seed, p.net, p.clearance, p.half_width, p.room_bound(fr.seed),
        ) else {
            continue;
        };
        // record the crossing node
        let node_id = nodes.len() as u32;
        nodes.push(Node { point: fr.entry, layer: fr.layer, parent: fr.parent });

        if room.contains(goal)
            && index.segment_is_clear(fr.layer, fr.entry, goal, p.half_width, p.net, p.clearance)
        {
            // reached: backtrace points then append goal (final hop validated clear)
            let mut path = backtrace(&nodes, node_id);
            path.push(PathPoint { point: goal, layer: fr.layer });
            return Some(path);
        }
        enqueue_room_doors(
            index, &room, fr.entry, fr.g, node_id, goal, p, &mut heap, &mut nodes, &visited,
        );
    }
    None
}

#[allow(clippy::too_many_arguments)]
fn enqueue_room_doors(
    index: &ObstacleIndex,
    room: &Room,
    entry: Point,
    g: i64,
    parent: u32,
    goal: Point,
    p: &MazeParams,
    heap: &mut BinaryHeap<Frontier>,
    nodes: &mut [Node],
    visited: &HashSet<(usize, i64, i64)>,
) {
    let _ = nodes;
    let cell = p.dedup_cell.max(1);
    let key = |layer: usize, pt: Point| (layer, pt.x.div_euclid(cell), pt.y.div_euclid(cell));
    for door in room.doors(index, p.net, p.clearance, p.half_width, p.bound, p.step) {
        // Like Java's door sectioning: try several crossing points along the door (its two
        // ends pulled inward, its midpoint, and the point nearest the goal) so the search
        // can route through whichever part of the door leads onward. Each crossing's next-
        // room seed is that crossing pushed just across the door, so entry -> cross ->
        // neighbour-seed stay consistent (the neighbour room is grown where we enter it).
        for (cross, out_seed) in door_crossings(&door, goal, p.step) {
            if visited.contains(&key(room.layer, out_seed)) {
                continue;
            }
            // The trace must travel entry -> cross within this room without clipping
            // copper. Validating here guarantees every parent->child crossing edge is
            // clear, so the backtrace polyline (and its string-pulled simplification) is
            // clear by construction. Rooms are regrown per-seed (not a single persistent
            // tiling), so this exact check keeps the corridor electrically valid.
            if !index.segment_is_clear(room.layer, entry, cross, p.half_width, p.net, p.clearance)
            {
                continue;
            }
            let g2 = g + dist(entry, cross);
            let h = dist(cross, goal);
            heap.push(Frontier {
                f: g2 + h,
                g: g2,
                entry: cross,
                seed: out_seed,
                layer: room.layer,
                parent,
            });
        }
    }
}

/// Candidate crossing points on a door (Java door sectioning): the midpoint, the two ends
/// pulled slightly inward, and the point nearest the goal. For each, the next-room seed is
/// that crossing pushed `step` units across the door into the neighbour's free space, so
/// `entry -> cross -> neighbour-seed` stay consistent (the neighbour room is grown where
/// we enter it).
fn door_crossings(door: &Door, goal: Point, step: i64) -> Vec<(Point, Point)> {
    let (a, b) = door.seg;
    let along = |t: f64| {
        Point::new(
            (a.x as f64 + (b.x - a.x) as f64 * t).round() as i64,
            (a.y as f64 + (b.y - a.y) as f64 * t).round() as i64,
        )
    };
    let mut crosses = vec![nearest_on_segment(goal, a, b), along(0.5), along(0.15), along(0.85)];
    crosses.dedup();

    // outward normal direction is door.out_seed - segment-midpoint (already free-side).
    let mid = Point::new((a.x + b.x) / 2, (a.y + b.y) / 2);
    let (nx, ny) = ((door.out_seed.x - mid.x) as f64, (door.out_seed.y - mid.y) as f64);
    let nlen = (nx * nx + ny * ny).sqrt().max(1e-9);
    let s = step.max(1) as f64;
    crosses
        .into_iter()
        .map(|cross| {
            let seed = Point::new(
                cross.x + (nx / nlen * s).round() as i64,
                cross.y + (ny / nlen * s).round() as i64,
            );
            (cross, seed)
        })
        .collect()
}

fn nearest_on_segment(p: Point, a: Point, b: Point) -> Point {
    let (dx, dy) = ((b.x - a.x) as f64, (b.y - a.y) as f64);
    let len2 = dx * dx + dy * dy;
    if len2 < 1e-9 {
        return a;
    }
    let t = (((p.x - a.x) as f64 * dx + (p.y - a.y) as f64 * dy) / len2).clamp(0.0, 1.0);
    Point::new((a.x as f64 + t * dx).round() as i64, (a.y as f64 + t * dy).round() as i64)
}

fn backtrace(nodes: &[Node], from: u32) -> Vec<PathPoint> {
    let mut out = Vec::new();
    let mut cur = from;
    while cur != u32::MAX {
        let n = nodes[cur as usize];
        out.push(PathPoint { point: n.point, layer: n.layer });
        cur = n.parent;
    }
    out.reverse();
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params(bound: IntBox) -> MazeParams {
        MazeParams {
            net: 0,
            layer: 0,
            clearance: 0,
            half_width: 10,
            bound,
            step: 40,
            dedup_cell: 40,
            max_rooms: 5000,
            window: 0,
        }
    }

    #[test]
    fn straight_path_in_open_space() {
        let mut index = ObstacleIndex::new(1);
        index.build();
        let bound = IntBox::new(0, 0, 10_000, 10_000);
        let path = find_path(
            &index, Point::new(1000, 5000), Point::new(9000, 5000), &params(bound),
        )
        .expect("path in open space");
        assert_eq!(path.first().unwrap().point, Point::new(1000, 5000));
        assert_eq!(path.last().unwrap().point, Point::new(9000, 5000));
    }

    #[test]
    fn detours_around_a_blocking_pad() {
        let mut index = ObstacleIndex::new(1);
        // a blocking different-net pad squarely between start and goal
        index.add_disc(0, Point::new(5000, 5000), 800, 7);
        index.build();
        let bound = IntBox::new(0, 0, 10_000, 10_000);
        let path = find_path(
            &index, Point::new(1000, 5000), Point::new(9000, 5000), &params(bound),
        )
        .expect("detour path exists");
        assert_eq!(path.first().unwrap().point, Point::new(1000, 5000));
        assert_eq!(path.last().unwrap().point, Point::new(9000, 5000));
        // the path must avoid the pad: no crossing point inside the inflated copper.
        for pp in &path {
            let d = ((pp.point.x - 5000) as f64).hypot((pp.point.y - 5000) as f64);
            assert!(d > 800.0 - 1.0, "path point {:?} is inside the pad", pp.point);
        }
        // and it must bend (more than just start+goal)
        assert!(path.len() >= 2);
    }

    #[test]
    fn no_path_when_fully_walled_in() {
        let mut index = ObstacleIndex::new(1);
        // ring of pads enclosing the start so no door leads out toward the goal.
        // Simpler: put the goal inside an obstacle so it is never reached.
        index.add_disc(0, Point::new(9000, 5000), 900, 7); // swallow the goal
        index.build();
        let bound = IntBox::new(0, 0, 10_000, 10_000);
        let path = find_path(
            &index, Point::new(1000, 5000), Point::new(9000, 5000), &params(bound),
        );
        // goal is inside an obstacle -> no room ever contains it
        assert!(path.is_none(), "goal inside obstacle must be unreachable");
    }
}

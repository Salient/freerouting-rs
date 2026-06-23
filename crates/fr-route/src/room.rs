//! Free-space expansion rooms (room/door model; see ROOMDOOR_DESIGN.md).
//!
//! Faithful port of the Java `ShapeSearchTree.complete_shape` / `restrain_shape`: a room
//! is the maximal convex tile of obstacle-free space that still contains a seed
//! ("contained") shape on one layer. We start from a bounding box and, for every nearby
//! different-net obstacle (represented as a convex tile = its copper inflated by clearance
//! + trace half-width), clip the room against the obstacle's border lines.
//!
//! Crucially this mirrors the Java algorithm exactly:
//!   * For an obstacle that intersects the room interior, find the obstacle border line
//!     that the seed shape is entirely on the LEFT of and that is FURTHEST from the seed;
//!     clip the room to the opposite (inner) half-plane of that line.
//!   * If no single line separates the whole seed (the obstacle cuts through the seed),
//!     pick a line that crosses the seed, split the room into the two half-planes, keep
//!     the piece on the seed side as one room, and recurse on the rest. This yields
//!     several rooms tiling the free space — the behaviour the maze search relies on so
//!     adjacent rooms share exact edges (and thus doors).
//!
//! Obstacle copper is inflated by `clearance + half_width` so a trace of that half-width
//! routed anywhere in the room clears all copper by `clearance`. Discs are approximated by
//! a regular octagon circumscribing the inflated circle (so the room never overlaps the
//! true circle); segments by their inflated convex hull (an octagon-capped rectangle).

use fr_geometry::{ConvexTile, IntBox, Point, Side};
use fr_spatial::{ObstacleIndex, ObstacleShape};

/// A convex free-space room on a single layer.
#[derive(Clone, Debug)]
pub struct Room {
    pub layer: usize,
    pub tile: ConvexTile,
    /// The seed ("contained") shape the room was grown around (always inside the tile).
    pub seed: ConvexTile,
}

impl Room {
    pub fn contains(&self, p: Point) -> bool {
        self.tile.contains(p)
    }
}

/// A door: a border edge of a room that faces FREE space (not a wall on an obstacle or the
/// global bound). The maze search steps from one room to a neighbour through a door. The
/// `seg` is the shared boundary segment (used for backtrace); `out_seed` is a point just
/// across the door in the neighbour's free space (used to complete the neighbour room).
#[derive(Clone, Copy, Debug)]
pub struct Door {
    pub seg: (Point, Point),
    pub out_seed: Point,
}

impl Room {
    /// The doors of this room: contiguous free-space sub-segments of the room's border.
    ///
    /// A room edge is part wall, part door: where the outward side is the global bound or
    /// a different-net obstacle's inflated copper it is a wall; the remaining sub-segments
    /// face free space (an adjacent free room) and are doors. We find them by sampling each
    /// edge every ~`step` units and grouping consecutive free samples into door segments —
    /// a correctness-equivalent optimization over the Java sorted-neighbour gap filling
    /// (which computes the same free sub-segments analytically). Each door's `out_seed` is
    /// a free point just across the edge, used to grow the neighbour room.
    pub fn doors(
        &self,
        index: &ObstacleIndex,
        net: u32,
        clearance: i64,
        half_width: i64,
        bound: IntBox,
        step: i64,
    ) -> Vec<Door> {
        let margin = (clearance + half_width).max(0);
        let step = step.max(1);
        // probe a bit further out than the edge so we land in the neighbour interior.
        let probe_out = (step / 2).max(1);
        let mut out = Vec::new();
        let n = self.tile.border_line_count();
        for i in 0..n {
            let (a, b) = self.tile.border_line(i);
            let (dx, dy) = ((b.x - a.x) as f64, (b.y - a.y) as f64);
            let len = (dx * dx + dy * dy).sqrt();
            if len < 1e-9 {
                continue;
            }
            // outward normal of a CCW edge a->b is (dy, -dx) (right of the edge).
            let (nx, ny) = (dy / len, -dx / len);
            let samples = (len / step as f64).floor() as i64 + 1;
            // group consecutive free samples into sub-segment [run_start_t, run_end_t].
            let mut run_start: Option<f64> = None;
            let mut last_free_t = 0.0f64;
            let emit = |t0: f64, t1: f64, out: &mut Vec<Door>| {
                let sa = Point::new(
                    (a.x as f64 + dx * t0).round() as i64,
                    (a.y as f64 + dy * t0).round() as i64,
                );
                let sb = Point::new(
                    (a.x as f64 + dx * t1).round() as i64,
                    (a.y as f64 + dy * t1).round() as i64,
                );
                let mt = 0.5 * (t0 + t1);
                let m = Point::new(
                    (a.x as f64 + dx * mt).round() as i64,
                    (a.y as f64 + dy * mt).round() as i64,
                );
                let seed = Point::new(
                    m.x + (nx * probe_out as f64).round() as i64,
                    m.y + (ny * probe_out as f64).round() as i64,
                );
                out.push(Door { seg: (sa, sb), out_seed: seed });
            };
            for s in 0..=samples {
                let t = if samples == 0 { 0.0 } else { s as f64 / samples as f64 };
                let on = Point::new(
                    (a.x as f64 + dx * t).round() as i64,
                    (a.y as f64 + dy * t).round() as i64,
                );
                let probe = Point::new(
                    on.x + (nx * probe_out as f64).round() as i64,
                    on.y + (ny * probe_out as f64).round() as i64,
                );
                let free = bound.contains(probe)
                    && !point_blocked(index, self.layer, probe, net, margin);
                if free {
                    if run_start.is_none() {
                        run_start = Some(t);
                    }
                    last_free_t = t;
                } else if let Some(t0) = run_start.take() {
                    emit(t0, last_free_t, &mut out);
                }
            }
            if let Some(t0) = run_start.take() {
                emit(t0, last_free_t, &mut out);
            }
        }
        out
    }
}

/// True if `p` is within `margin` of (i.e. inside the inflated copper of) some different-
/// net obstacle on `layer`. Uses a zero-length segment clearance query.
fn point_blocked(index: &ObstacleIndex, layer: usize, p: Point, net: u32, margin: i64) -> bool {
    match index.min_clearance_margin_within(layer, p, p, 0, net, margin.max(1)) {
        Some(gap) => gap < margin as f64,
        None => false,
    }
}

/// Build the free-space room(s) around `seed_pt` on `layer`. `clearance + half_width` is
/// the margin kept from every different-net obstacle. Returns every maximal convex room
/// whose interior is obstacle-free and that contains part of the seed; the first room in
/// the result is the one containing `seed_pt` itself (the others tile the rest of the
/// region carved by an obstacle that split the seed).
///
/// Returns an empty vec if `seed_pt` is inside an obstacle's inflated copper.
pub fn complete_rooms(
    index: &ObstacleIndex,
    layer: usize,
    seed_pt: Point,
    net: u32,
    clearance: i64,
    half_width: i64,
    bound: IntBox,
) -> Vec<Room> {
    if !bound.contains(seed_pt) {
        return Vec::new();
    }
    let margin = (clearance + half_width).max(0);

    // The seed shape: a tiny box around the seed point (the Java `contained_shape`; for a
    // point start it is a trace-width square). Keep it small but non-degenerate so the
    // "side_of line" tests behave. One board unit suffices for exactness.
    let s = 1;
    let seed = ConvexTile::from_box(IntBox::new(
        seed_pt.x - s, seed_pt.y - s, seed_pt.x + s, seed_pt.y + s,
    ));

    // Gather nearby obstacle tiles (inflated copper). Query a box covering the bound. (A
    // smaller local cap was tried for speed but can exclude an obstacle that should bound
    // the room — keep the full bound for correctness; the bound is already the windowed
    // neighbourhood from MazeParams::room_bound for the maze caller.)
    let reach = (bound.width().max(bound.height())).max(1);
    let qbox = IntBox::new(
        seed_pt.x - reach, seed_pt.y - reach, seed_pt.x + reach, seed_pt.y + reach,
    );
    let obstacles: Vec<ConvexTile> = index
        .query_box(layer, qbox, net)
        .into_iter()
        .map(|(shape, _net)| inflate_obstacle(&shape, margin))
        .collect();

    // Reject immediately if the seed point is inside any inflated obstacle.
    for ob in &obstacles {
        if ob.contains(seed_pt) {
            return Vec::new();
        }
    }

    // Start with the full bound as one incomplete room, then restrain against each
    // obstacle in turn (Java complete_shape's leaf loop).
    let mut rooms: Vec<(ConvexTile, ConvexTile)> =
        vec![(ConvexTile::from_box(bound), seed.clone())];
    for ob in &obstacles {
        let mut next: Vec<(ConvexTile, ConvexTile)> = Vec::new();
        for (room_tile, contained) in rooms.drain(..) {
            let inter = room_tile.intersection(ob);
            if inter.dimension() == 2 {
                // obstacle overlaps this room's interior: restrain.
                restrain_shape(&room_tile, ob, &contained, &mut next);
            } else {
                next.push((room_tile, contained));
            }
        }
        rooms = next;
    }

    // Keep only 2-D rooms; order so the room containing the seed point is first.
    let mut out: Vec<Room> = rooms
        .into_iter()
        .filter(|(t, _)| t.dimension() == 2)
        .map(|(tile, seed)| Room { layer, tile, seed })
        .collect();
    out.sort_by_key(|r| if r.tile.contains(seed_pt) { 0 } else { 1 });
    out
}

/// Convenience: just the room containing the seed point, if any.
pub fn complete_room(
    index: &ObstacleIndex,
    layer: usize,
    seed_pt: Point,
    net: u32,
    clearance: i64,
    half_width: i64,
    bound: IntBox,
) -> Option<Room> {
    complete_rooms(index, layer, seed_pt, net, clearance, half_width, bound)
        .into_iter()
        .find(|r| r.tile.contains(seed_pt))
}

/// Restrain `room` so it does not overlap the interior of `obstacle`, keeping `contained`
/// inside the result. Pushes the resulting room(s) (tile, contained) into `out`. Faithful
/// port of Java `ShapeSearchTree.restrain_shape`.
fn restrain_shape(
    room: &ConvexTile,
    obstacle: &ConvexTile,
    contained: &ConvexTile,
    out: &mut Vec<(ConvexTile, ConvexTile)>,
) {
    if contained.is_empty() {
        return;
    }
    // 1. Find the obstacle border line that (a) intersects the room interior and (b) the
    //    contained shape is entirely on its LEFT; take the one FURTHEST to the left of the
    //    contained shape. Clip the room to that line's OPPOSITE (inner) half-plane.
    let mut cut: Option<(Point, Point)> = None; // directed line (opposite of the obstacle edge)
    let mut cut_distance = -1.0f64;
    let n = obstacle.border_line_count();
    for i in 0..n {
        let (la, lb) = obstacle.border_line(i);
        if !room.line_intersects_interior(la, lb) {
            continue;
        }
        // distance_to_the_left: min signed distance of `contained` corners to the line,
        // or -1 if any corner is on the RIGHT (i.e. contained is not fully left).
        if let Some(d) = distance_to_the_left(contained, la, lb) {
            if d > cut_distance {
                cut_distance = d;
                // clip to the OPPOSITE half-plane: the inner side keeping the room. The
                // obstacle edge (la->lb) is CCW so its interior (the obstacle) is on the
                // LEFT; the room must stay on the RIGHT, i.e. the LEFT of the reversed
                // line lb->la. clip_halfplane keeps the LEFT side, so use (lb, la).
                cut = Some((lb, la));
            }
        }
    }
    if let Some((a, b)) = cut {
        let piece = room.clip_halfplane(a, b);
        if piece.dimension() >= 2 {
            out.push((piece, contained.clone()));
        }
        return;
    }

    // 2. No separating line: the obstacle cuts through the contained shape. Find a border
    //    line crossing the contained interior, split the room, keep the seed-side piece,
    //    and recurse on the rest (Java's second branch).
    if contained.dimension() < 1 {
        return; // a completed room already surrounds the point seed
    }
    for i in 0..n {
        let (la, lb) = obstacle.border_line(i);
        if !room.line_intersects_interior(la, lb) {
            continue;
        }
        if contained.line_intersects_interior(la, lb) {
            // cut_line = opposite of this obstacle edge: keep the room on the room side
            // (left of lb->la). The contained shape is split across this line.
            let (a, b) = (lb, la);
            let keep = room.clip_halfplane(a, b);
            let keep_seed = contained.clip_halfplane(a, b);
            if keep.dimension() >= 2 {
                out.push((keep, keep_seed));
            }
            // rest = the other half (left of la->lb); recurse against the same obstacle.
            let rest = room.clip_halfplane(la, lb);
            let rest_seed = contained.clip_halfplane(la, lb);
            if rest.dimension() >= 2 && !rest_seed.is_empty() {
                restrain_shape(&rest, obstacle, &rest_seed, out);
            }
            return;
        }
    }
    // No cut line found: region already occupied elsewhere; drop (Java returns empty).
}

/// Min signed distance from line a->b to the corners of `tile`, treating LEFT as positive;
/// returns None if any corner is strictly on the RIGHT of a->b (i.e. the tile is not fully
/// on the left). Mirrors Java `TileShape.distance_to_the_left` (which measures distance to
/// a line that the shape is left of). Here a->b is the obstacle edge (CCW, interior left),
/// and "tile fully on the right of the edge" == "fully on the left of the reversed edge".
fn distance_to_the_left(tile: &ConvexTile, a: Point, b: Point) -> Option<f64> {
    // We want: contained entirely on the RIGHT of the obstacle edge a->b (outside the
    // obstacle). Distance = min distance of contained corners to the edge line.
    let mut result = f64::MAX;
    for &c in tile.vertices() {
        match a.side_of(b, c) {
            Side::Left => return None, // a corner is inside the obstacle half-plane
            _ => {}
        }
        result = result.min(point_line_distance(c, a, b));
    }
    if result == f64::MAX {
        None
    } else {
        Some(result)
    }
}

/// Perpendicular distance from point p to the infinite line through a->b.
fn point_line_distance(p: Point, a: Point, b: Point) -> f64 {
    let dx = (b.x - a.x) as f64;
    let dy = (b.y - a.y) as f64;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-9 {
        let px = (p.x - a.x) as f64;
        let py = (p.y - a.y) as f64;
        return (px * px + py * py).sqrt();
    }
    // |cross| / len
    let cross = (b.x - a.x) as f64 * (p.y - a.y) as f64 - (b.y - a.y) as f64 * (p.x - a.x) as f64;
    cross.abs() / len
}

/// Build a convex tile from inflating an obstacle's copper by `margin`. Disc -> octagon
/// circumscribing the inflated circle (so the tile fully covers the true inflated copper);
/// segment -> the inflated stadium approximated by an octagon-ish convex hull.
fn inflate_obstacle(shape: &ObstacleShape, margin: i64) -> ConvexTile {
    match *shape {
        ObstacleShape::Disc { center, radius } => octagon(center, radius + margin),
        ObstacleShape::Seg { a, b, half } => inflate_segment(a, b, half + margin),
    }
}

/// Regular octagon circumscribing a circle of radius `r` centered at `c` (each side
/// tangent to the circle, so the octagon CONTAINS the circle). CCW.
fn octagon(c: Point, r: i64) -> ConvexTile {
    let r = r.max(1);
    // circumscribing octagon: the orthogonal sides are at distance r; the diagonal sides
    // at distance r too. Half-width across flats = r; corner offset = r * tan(22.5) ~
    // 0.4142*r for the bevel. Use the standard "rounded square" octagon with corner cut
    // k = round(r * (sqrt(2)-1)).
    let k = ((r as f64) * (std::f64::consts::SQRT_2 - 1.0)).round() as i64;
    let (x, y) = (c.x, c.y);
    ConvexTile::from_ccw(vec![
        Point::new(x + r, y - k),
        Point::new(x + r, y + k),
        Point::new(x + k, y + r),
        Point::new(x - k, y + r),
        Point::new(x - r, y + k),
        Point::new(x - r, y - k),
        Point::new(x - k, y - r),
        Point::new(x + k, y - r),
    ])
}

/// Inflate a segment a-b by `r` into a convex tile (its Minkowski sum with a disc,
/// approximated by an octagon-capped hull). For simplicity and convexity we take the
/// convex hull of two circumscribing octagons placed at the endpoints.
fn inflate_segment(a: Point, b: Point, r: i64) -> ConvexTile {
    let oa = octagon(a, r);
    let ob = octagon(b, r);
    let mut pts: Vec<Point> = oa.vertices().to_vec();
    pts.extend_from_slice(ob.vertices());
    convex_hull(&pts)
}

/// Andrew's monotone-chain convex hull (CCW), exact integer.
fn convex_hull(pts: &[Point]) -> ConvexTile {
    let mut p: Vec<Point> = pts.to_vec();
    p.sort_by(|u, v| u.x.cmp(&v.x).then(u.y.cmp(&v.y)));
    p.dedup();
    if p.len() < 3 {
        return ConvexTile::from_ccw(p);
    }
    let cross = |o: Point, a: Point, b: Point| -> i128 {
        (a.x - o.x) as i128 * (b.y - o.y) as i128 - (a.y - o.y) as i128 * (b.x - o.x) as i128
    };
    let mut lower: Vec<Point> = Vec::new();
    for &pt in &p {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], pt) <= 0 {
            lower.pop();
        }
        lower.push(pt);
    }
    let mut upper: Vec<Point> = Vec::new();
    for &pt in p.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], pt) <= 0 {
            upper.pop();
        }
        upper.push(pt);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    ConvexTile::from_ccw(lower)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn idx(layers: usize) -> ObstacleIndex {
        ObstacleIndex::new(layers)
    }

    #[test]
    fn empty_space_room_is_the_whole_bound() {
        let mut index = idx(1);
        index.build();
        let bound = IntBox::new(0, 0, 1000, 1000);
        let room = complete_room(&index, 0, Point::new(500, 500), 0, 0, 0, bound).unwrap();
        assert!(room.contains(Point::new(500, 500)));
        assert!(room.contains(Point::new(10, 10)));
        assert!(room.contains(Point::new(990, 990)));
        assert_eq!(room.tile.dimension(), 2);
    }

    #[test]
    fn room_excludes_a_pad() {
        let mut index = idx(1);
        index.add_disc(0, Point::new(800, 500), 50, 7);
        index.build();
        let bound = IntBox::new(0, 0, 1000, 1000);
        let room = complete_room(&index, 0, Point::new(300, 500), 0, 0, 10, bound).unwrap();
        assert!(room.contains(Point::new(300, 500)), "seed inside");
        assert!(!room.contains(Point::new(800, 500)), "pad center excluded");
        assert!(room.contains(Point::new(100, 500)), "seed side open");
    }

    #[test]
    fn seed_inside_obstacle_yields_no_room() {
        let mut index = idx(1);
        index.add_disc(0, Point::new(500, 500), 100, 7);
        index.build();
        let bound = IntBox::new(0, 0, 1000, 1000);
        assert!(complete_room(&index, 0, Point::new(500, 500), 0, 0, 10, bound).is_none());
    }

    #[test]
    fn room_between_two_pads_is_convex_and_open_vertically() {
        let mut index = idx(1);
        index.add_disc(0, Point::new(200, 500), 40, 7);
        index.add_disc(0, Point::new(800, 500), 40, 7);
        index.build();
        let bound = IntBox::new(0, 0, 1000, 1000);
        let room = complete_room(&index, 0, Point::new(500, 500), 0, 0, 10, bound).unwrap();
        assert!(room.contains(Point::new(500, 500)));
        assert!(!room.contains(Point::new(200, 500)));
        assert!(!room.contains(Point::new(800, 500)));
        assert!(room.tile.signed_area2() > 0, "CCW convex");
    }

    #[test]
    fn same_net_obstacle_does_not_clip() {
        let mut index = idx(1);
        index.add_disc(0, Point::new(800, 500), 50, 3);
        index.build();
        let bound = IntBox::new(0, 0, 1000, 1000);
        let room = complete_room(&index, 0, Point::new(300, 500), 3, 0, 10, bound).unwrap();
        assert!(room.contains(Point::new(800, 500)), "same-net copper not an obstacle");
    }

    #[test]
    fn room_does_not_overlap_inflated_pad_copper() {
        // The room tile must keep clear of the obstacle's inflated copper everywhere.
        let mut index = idx(1);
        index.add_disc(0, Point::new(700, 500), 60, 9);
        index.build();
        let bound = IntBox::new(0, 0, 1000, 1000);
        let room = complete_room(&index, 0, Point::new(200, 500), 0, 0, 20, bound).unwrap();
        // inflated radius = 60 + 20 = 80, edge at x=620 on the seed side. The room must
        // not contain a point inside the inflated copper.
        assert!(!room.contains(Point::new(680, 500)));
        assert!(!room.contains(Point::new(700, 500)));
    }

    #[test]
    fn empty_room_doors_only_on_bound_edges_are_walls() {
        // A room equal to the full bound has all 4 edges on the bound -> no doors.
        let mut index = idx(1);
        index.build();
        let bound = IntBox::new(0, 0, 1000, 1000);
        let room = complete_room(&index, 0, Point::new(500, 500), 0, 0, 0, bound).unwrap();
        let doors = room.doors(&index, 0, 0, 0, bound, 5);
        assert!(doors.is_empty(), "a room filling the bound has only walls, got {}", doors.len());
    }

    #[test]
    fn room_clipped_by_pad_has_a_door_on_the_clip_edge() {
        // One pad to the right: the room's right boundary is a chord facing free space
        // around the pad -> at least one door; the pad-facing wall is not a door.
        let mut index = idx(1);
        index.add_disc(0, Point::new(800, 500), 40, 7);
        index.build();
        let bound = IntBox::new(0, 0, 1000, 1000);
        let room = complete_room(&index, 0, Point::new(300, 500), 0, 0, 10, bound).unwrap();
        let doors = room.doors(&index, 0, 0, 10, bound, 5);
        assert!(!doors.is_empty(), "expected at least one door facing free space");
        // every door's out_seed must be in free space (not inside the inflated pad)
        for d in &doors {
            assert!(!point_blocked(&index, 0, d.out_seed, 0, 50),
                "door out_seed must be free, got {:?}", d.out_seed);
        }
    }

    #[test]
    fn octagon_circumscribes_circle() {
        let oct = octagon(Point::new(0, 0), 100);
        assert_eq!(oct.dimension(), 2);
        // every circle point at radius 100 should be inside the circumscribing octagon;
        // check the 8 axis/diagonal extremes.
        for &(dx, dy) in &[(100, 0), (0, 100), (-100, 0), (0, -100), (70, 70), (-70, -70)] {
            assert!(oct.contains(Point::new(dx, dy)), "circle point ({dx},{dy}) inside octagon");
        }
    }
}

//! Free-space expansion rooms (Stage 1 of the room/door model; see ROOMDOOR_DESIGN.md).
//!
//! A room is a maximal convex tile of obstacle-free space on one layer, containing a seed
//! point. We build it the way the Java `CompleteFreeSpaceExpansionRoom` does in spirit:
//! start from a bounding box (the board/search region) and clip it against a separating
//! half-plane for every nearby different-net obstacle, so the resulting convex tile holds
//! the seed and excludes all obstacle copper inflated by the clearance + trace half-width.
//!
//! Clipping uses exact `ConvexTile::clip_halfplane`; obstacle geometry comes from the
//! exact `fr_spatial::ObstacleIndex`. The room is the any-angle (`Simplex`) variant; the
//! 45/90-degree variants (Stage 5) will additionally snap the clip lines.

use fr_geometry::{ConvexTile, FloatPoint, IntBox, Point};
use fr_spatial::{ObstacleIndex, ObstacleShape};

/// A convex free-space room on a single layer.
#[derive(Clone, Debug)]
pub struct Room {
    pub layer: usize,
    pub tile: ConvexTile,
    /// The seed point the room was grown around (always inside the tile).
    pub seed: Point,
}

impl Room {
    pub fn contains(&self, p: Point) -> bool {
        self.tile.contains(p)
    }
}

/// Build the maximal convex free room around `seed` on `layer`, clipping `bound` against a
/// separating half-plane for each different-net obstacle whose inflated copper is near the
/// seed. `clearance` + `half_width` is the margin kept from every obstacle (so a trace of
/// the given half-width routed anywhere in the room clears all copper by `clearance`).
///
/// Returns `None` if the seed itself is inside an obstacle's inflated copper (no room) or
/// the clip collapses the tile to empty.
pub fn complete_room(
    index: &ObstacleIndex,
    layer: usize,
    seed: Point,
    net: u32,
    clearance: i64,
    half_width: i64,
    bound: IntBox,
) -> Option<Room> {
    if !bound.contains(seed) {
        return None;
    }
    let mut tile = ConvexTile::from_box(bound);
    let margin = (clearance + half_width).max(0);

    // Query a generous box around the seed: obstacles whose inflated copper could bound a
    // room containing the seed. We expand by the bound extent so far-but-large obstacles
    // (e.g. long traces) are still considered; the R-tree keeps this cheap.
    let reach = (bound.width().max(bound.height())).max(1);
    let qbox = IntBox::new(seed.x - reach, seed.y - reach, seed.x + reach, seed.y + reach);
    let obstacles = index.query_box(layer, qbox, net);

    let seed_f = seed.to_float();
    for (shape, _onet) in obstacles {
        // Effective radius of the obstacle's inflated copper as seen from the seed: its
        // half-extent plus the margin. The separating line is perpendicular to the
        // seed->obstacle direction, tangent to the inflated copper on the seed side.
        let inflate = (shape.half_extent() + margin) as f64;
        let center_dist = shape.center_dist_to_point(seed);
        if center_dist <= inflate + 1.0 {
            // The seed is inside (or on) this obstacle's inflated copper: no valid room.
            return None;
        }
        // Direction from the obstacle's nearest reference point to the seed.
        let toward = obstacle_dir_toward(&shape, seed_f);
        let Some((dx, dy)) = toward else { continue };
        // The tangent point: step from the seed back toward the obstacle by
        // (center_dist - inflate) along the unit direction; the clip line passes through
        // it, perpendicular to (dx,dy), keeping the seed side.
        let back = center_dist - inflate;
        let tangent = FloatPoint::new(seed_f.x - dx * back, seed_f.y - dy * back);
        // A directed line through `tangent` perpendicular to (dx,dy) such that the seed is
        // on the LEFT (kept) side. Direction along the line is (-dy, dx) gives left =
        // +(dx,dy) side; pick endpoints far apart for an exact integer half-plane.
        let big = (reach as f64) * 4.0 + 1.0;
        let la = Point::new(
            (tangent.x - (-dy) * big).round() as i64,
            (tangent.y - dx * big).round() as i64,
        );
        let lb = Point::new(
            (tangent.x + (-dy) * big).round() as i64,
            (tangent.y + dx * big).round() as i64,
        );
        // clip_halfplane keeps the LEFT/On side of a->b. Choose a,b so the seed is left.
        let (a, b) = if left_of(la, lb, seed) { (la, lb) } else { (lb, la) };
        tile = tile.clip_halfplane(a, b);
        if tile.is_empty() {
            return None;
        }
    }

    if tile.is_empty() || !tile.contains(seed) {
        return None;
    }
    Some(Room { layer, tile, seed })
}

/// Unit direction (dx,dy) pointing from the obstacle toward the seed. For a disc this is
/// from the center; for a segment, from the nearest point on the segment.
fn obstacle_dir_toward(shape: &ObstacleShape, seed: FloatPoint) -> Option<(f64, f64)> {
    let from = match *shape {
        ObstacleShape::Disc { center, .. } => center.to_float(),
        ObstacleShape::Seg { a, b, .. } => nearest_on_seg(seed, a.to_float(), b.to_float()),
    };
    let dx = seed.x - from.x;
    let dy = seed.y - from.y;
    let len = (dx * dx + dy * dy).sqrt();
    if len < 1e-9 {
        return None;
    }
    Some((dx / len, dy / len))
}

fn nearest_on_seg(p: FloatPoint, a: FloatPoint, b: FloatPoint) -> FloatPoint {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len2 = dx * dx + dy * dy;
    if len2 < 1e-9 {
        return a;
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len2).clamp(0.0, 1.0);
    FloatPoint::new(a.x + t * dx, a.y + t * dy)
}

/// True if `p` is strictly left of (or on) the directed line a->b.
fn left_of(a: Point, b: Point, p: Point) -> bool {
    a.side_of(b, p) != fr_geometry::Side::Right
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
        // no obstacles: the room is the full bounding box (4 corners)
        assert!(room.contains(Point::new(500, 500)));
        assert!(room.contains(Point::new(10, 10)));
        assert!(room.contains(Point::new(990, 990)));
        assert_eq!(room.tile.vertices().len(), 4);
    }

    #[test]
    fn room_excludes_a_pad() {
        let mut index = idx(1);
        // a different-net pad to the right of the seed
        index.add_disc(0, Point::new(800, 500), 50, 7);
        index.build();
        let bound = IntBox::new(0, 0, 1000, 1000);
        let room = complete_room(&index, 0, Point::new(300, 500), 0, 0, 10, bound).unwrap();
        assert!(room.contains(Point::new(300, 500)), "seed inside");
        // the pad copper (center 800, r50) inflated by half 10 -> edge at x=740; the room
        // must NOT contain a point well inside that inflated copper
        assert!(!room.contains(Point::new(800, 500)), "pad center excluded");
        assert!(!room.contains(Point::new(770, 500)), "inflated pad excluded");
        // a point on the seed side is still in
        assert!(room.contains(Point::new(100, 500)));
    }

    #[test]
    fn seed_inside_obstacle_yields_no_room() {
        let mut index = idx(1);
        index.add_disc(0, Point::new(500, 500), 100, 7);
        index.build();
        let bound = IntBox::new(0, 0, 1000, 1000);
        // seed at the pad center, inside its copper
        assert!(complete_room(&index, 0, Point::new(500, 500), 0, 0, 10, bound).is_none());
    }

    #[test]
    fn room_is_convex_and_between_two_pads() {
        let mut index = idx(1);
        index.add_disc(0, Point::new(200, 500), 40, 7); // left
        index.add_disc(0, Point::new(800, 500), 40, 7); // right
        index.build();
        let bound = IntBox::new(0, 0, 1000, 1000);
        let room = complete_room(&index, 0, Point::new(500, 500), 0, 0, 10, bound).unwrap();
        assert!(room.contains(Point::new(500, 500)));
        // both inflated pads excluded
        assert!(!room.contains(Point::new(200, 500)));
        assert!(!room.contains(Point::new(800, 500)));
        // a vertical strip through the seed remains open
        assert!(room.contains(Point::new(500, 100)));
        assert!(room.contains(Point::new(500, 900)));
        // convex (CCW, positive area)
        assert!(room.tile.signed_area2() > 0);
    }

    #[test]
    fn same_net_obstacle_does_not_clip() {
        let mut index = idx(1);
        index.add_disc(0, Point::new(800, 500), 50, 3); // SAME net as the route
        index.build();
        let bound = IntBox::new(0, 0, 1000, 1000);
        let room = complete_room(&index, 0, Point::new(300, 500), 3, 0, 10, bound).unwrap();
        // same-net copper is not an obstacle: the room can extend over it
        assert!(room.contains(Point::new(800, 500)));
    }
}

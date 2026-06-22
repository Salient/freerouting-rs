//! Backtrace a room/door path into a clean any-angle trace polyline (room/door stage 4).
//!
//! The Java `LocateFoundConnectionAlgoAnyAngle` walks the door sequence and pulls the
//! trace taut with a tangent-visibility sweep, producing the minimum-bend free-angle
//! polyline that stays inside the room corridor. We achieve the equivalent result with a
//! string-pulling straightener validated against the exact obstacle geometry: starting
//! from the first point, we extend each straight run as far along the path as the direct
//! segment stays clear of all different-net copper (by `clearance`), then bend. This keeps
//! the trace any-angle and minimal-bend while guaranteeing the same electrical clearance
//! the grid router's exact validator enforces (so DRC stays 0/0).

use fr_geometry::{FloatPoint, Point};
use fr_spatial::ObstacleIndex;

use crate::maze::PathPoint;

/// Trace angle restriction (the GUI "snap angle" setting). Mirrors the Java
/// `AngleRestriction`: NONE = any angle, FORTYFIVE = 0/45/90 only, NINETY = 0/90 only.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AngleRestriction {
    None,
    FortyFive,
    Ninety,
}

/// Straighten a room/door crossing path into a minimal-bend any-angle polyline on a single
/// layer, keeping every segment clear of different-net copper by `clearance`. Input points
/// must share one layer (stage 4 is single-layer; vias handled in stage 6). Returns the
/// simplified corner list (>= 2 points), or None if the input is degenerate.
pub fn straighten(
    index: &ObstacleIndex,
    path: &[PathPoint],
    net: u32,
    half_width: i64,
    clearance: i64,
) -> Option<Vec<Point>> {
    if path.len() < 2 {
        return None;
    }
    let layer = path[0].layer;
    let pts: Vec<Point> = path.iter().map(|p| p.point).collect();

    // String-pulling: from the current anchor, find the farthest path index reachable by a
    // single clear straight segment; emit it as a corner and continue from there. Greedy
    // farthest-clear is the discrete analogue of the tangent-visibility sweep. The maze
    // guarantees every consecutive pair is clear-connectable, so `best` is always >=
    // anchor+1 with a clear segment — string-pulling only ever removes corners, never
    // introduces a clipping one.
    let mut out = vec![pts[0]];
    let mut anchor = 0usize;
    while anchor + 1 < pts.len() {
        let mut best = anchor + 1; // consecutive hop is guaranteed clear by the maze
        for j in (anchor + 2)..pts.len() {
            if segment_clear(index, layer, pts[anchor], pts[j], net, half_width, clearance) {
                best = j;
            }
        }
        out.push(pts[best]);
        anchor = best;
    }

    Some(simplify_collinear(&out))
}

fn segment_clear(
    index: &ObstacleIndex,
    layer: usize,
    a: Point,
    b: Point,
    net: u32,
    half_width: i64,
    clearance: i64,
) -> bool {
    index.segment_is_clear(layer, a, b, half_width, net, clearance)
}

/// Straighten, then snap to the given angle restriction. For each segment of the any-angle
/// polyline we insert one elbow corner (Java `calculate_additional_corner`) so the two
/// resulting sub-segments obey the restriction (90: axis-aligned; 45: axis or diagonal).
/// Both `horizontal_first` orientations are tried and the elbow is accepted only if BOTH
/// sub-segments stay clear; if neither orientation is clear, the original any-angle segment
/// is kept (so snapping never makes a trace worse or non-clear). NONE returns the
/// any-angle polyline unchanged.
pub fn straighten_angled(
    index: &ObstacleIndex,
    path: &[PathPoint],
    net: u32,
    half_width: i64,
    clearance: i64,
    angle: AngleRestriction,
) -> Option<Vec<Point>> {
    let base = straighten(index, path, net, half_width, clearance)?;
    if angle == AngleRestriction::None || base.len() < 2 {
        return Some(base);
    }
    let layer = path[0].layer;
    let mut out = vec![base[0]];
    for w in base.windows(2) {
        let (from, to) = (w[0], w[1]);
        // try inserting an elbow for each orientation; keep the first whose sub-segments
        // are both clear.
        let mut placed = false;
        for horizontal_first in [true, false] {
            let elbow = additional_corner(from, to, horizontal_first, angle);
            if elbow == from || elbow == to {
                // already axis/diagonal aligned: no elbow needed
                if segment_clear(index, layer, from, to, net, half_width, clearance) {
                    out.push(to);
                    placed = true;
                    break;
                }
                continue;
            }
            if segment_clear(index, layer, from, elbow, net, half_width, clearance)
                && segment_clear(index, layer, elbow, to, net, half_width, clearance)
            {
                out.push(elbow);
                out.push(to);
                placed = true;
                break;
            }
        }
        if !placed {
            // snapping would clip: keep the any-angle segment (still clear by construction)
            out.push(to);
        }
    }
    Some(simplify_collinear(&out))
}

/// The elbow corner so that `from -> corner -> to` obeys the angle restriction. Port of
/// Java `calculate_additional_corner` (ninety_degree_corner / fortyfive_degree_corner).
fn additional_corner(from: Point, to: Point, horizontal_first: bool, angle: AngleRestriction) -> Point {
    let f = from.to_float();
    let t = to.to_float();
    let c = match angle {
        AngleRestriction::None => t,
        AngleRestriction::Ninety => ninety_corner(f, t, horizontal_first),
        AngleRestriction::FortyFive => fortyfive_corner(f, t, horizontal_first),
    };
    c.round()
}

fn ninety_corner(from: FloatPoint, to: FloatPoint, horizontal_first: bool) -> FloatPoint {
    if horizontal_first {
        FloatPoint::new(to.x, from.y)
    } else {
        FloatPoint::new(from.x, to.y)
    }
}

fn fortyfive_corner(from: FloatPoint, to: FloatPoint, horizontal_first: bool) -> FloatPoint {
    let abs_dx = (to.x - from.x).abs();
    let abs_dy = (to.y - from.y).abs();
    let (x, y);
    if abs_dx <= abs_dy {
        if horizontal_first {
            x = to.x;
            y = if to.y >= from.y { from.y + abs_dx } else { from.y - abs_dx };
        } else {
            x = from.x;
            y = if to.y > from.y { to.y - abs_dx } else { to.y + abs_dx };
        }
    } else if horizontal_first {
        y = from.y;
        x = if to.x > from.x { to.x - abs_dy } else { to.x + abs_dy };
    } else {
        y = to.y;
        x = if to.x > from.x { from.x + abs_dy } else { from.x - abs_dy };
    }
    FloatPoint::new(x, y)
}

/// Drop interior points collinear with their neighbours and consecutive duplicates.
fn simplify_collinear(pts: &[Point]) -> Vec<Point> {
    let mut dedup: Vec<Point> = Vec::with_capacity(pts.len());
    for &p in pts {
        if dedup.last() != Some(&p) {
            dedup.push(p);
        }
    }
    if dedup.len() <= 2 {
        return dedup;
    }
    let mut out = vec![dedup[0]];
    for i in 1..dedup.len() - 1 {
        let a = out[out.len() - 1];
        let b = dedup[i];
        let c = dedup[i + 1];
        if a.side_of(c, b) != fr_geometry::Side::On {
            out.push(b);
        }
    }
    out.push(*dedup.last().unwrap());
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::maze::{find_path, MazeParams};
    use fr_geometry::IntBox;

    fn pp(points: &[(i64, i64)]) -> Vec<PathPoint> {
        points.iter().map(|&(x, y)| PathPoint { point: Point::new(x, y), layer: 0 }).collect()
    }

    #[test]
    fn straightens_a_redundant_zigzag_in_open_space() {
        let mut index = ObstacleIndex::new(1);
        index.build();
        // a wandering path across open space should collapse to a single straight segment
        let path = pp(&[(0, 0), (100, 50), (200, -30), (300, 20), (400, 0)]);
        let line = straighten(&index, &path, 0, 10, 0).unwrap();
        assert_eq!(line.first().unwrap(), &Point::new(0, 0));
        assert_eq!(line.last().unwrap(), &Point::new(400, 0));
        assert_eq!(line.len(), 2, "open space collapses to a straight line, got {line:?}");
    }

    #[test]
    fn keeps_a_bend_around_an_obstacle() {
        let mut index = ObstacleIndex::new(1);
        index.add_disc(0, Point::new(200, 0), 60, 7); // blocks the straight line
        index.build();
        // path detours above the pad
        let path = pp(&[(0, 0), (200, 200), (400, 0)]);
        let line = straighten(&index, &path, 0, 10, 0).unwrap();
        assert_eq!(line.first().unwrap(), &Point::new(0, 0));
        assert_eq!(line.last().unwrap(), &Point::new(400, 0));
        // it cannot be a single straight segment (that would cross the pad)
        assert!(line.len() >= 3, "must keep a detour bend, got {line:?}");
        // and every segment must clear the pad
        for w in line.windows(2) {
            assert!(segment_clear(&index, 0, w[0], w[1], 0, 10, 0),
                "segment {:?} must clear the pad", w);
        }
    }

    /// classify a segment's direction: 0=axis (H or V), 1=45-diagonal, 2=other.
    fn seg_kind(a: Point, b: Point) -> u8 {
        let dx = (b.x - a.x).abs();
        let dy = (b.y - a.y).abs();
        if dx == 0 || dy == 0 {
            0
        } else if dx == dy {
            1
        } else {
            2
        }
    }

    #[test]
    fn ninety_degree_snaps_diagonal_to_L_bend() {
        let mut index = ObstacleIndex::new(1);
        index.build();
        let path = pp(&[(0, 0), (300, 200)]); // a diagonal in open space
        let line = straighten_angled(&index, &path, 0, 10, 0, AngleRestriction::Ninety).unwrap();
        assert_eq!(line.first().unwrap(), &Point::new(0, 0));
        assert_eq!(line.last().unwrap(), &Point::new(300, 200));
        // every segment must be axis-aligned
        for w in line.windows(2) {
            assert_eq!(seg_kind(w[0], w[1]), 0, "90-deg trace must be axis-aligned: {:?}", w);
        }
        assert!(line.len() >= 3, "an L-bend was inserted");
    }

    #[test]
    fn fortyfive_snaps_to_axis_or_diagonal() {
        let mut index = ObstacleIndex::new(1);
        index.build();
        let path = pp(&[(0, 0), (300, 100)]); // shallow diagonal
        let line = straighten_angled(&index, &path, 0, 10, 0, AngleRestriction::FortyFive).unwrap();
        assert_eq!(line.first().unwrap(), &Point::new(0, 0));
        assert_eq!(line.last().unwrap(), &Point::new(300, 100));
        for w in line.windows(2) {
            let k = seg_kind(w[0], w[1]);
            assert!(k == 0 || k == 1, "45-deg trace must be axis or diagonal: {:?} kind {k}", w);
        }
    }

    #[test]
    fn none_leaves_any_angle_untouched() {
        let mut index = ObstacleIndex::new(1);
        index.build();
        let path = pp(&[(0, 0), (300, 200)]);
        let line = straighten_angled(&index, &path, 0, 10, 0, AngleRestriction::None).unwrap();
        assert_eq!(line, vec![Point::new(0, 0), Point::new(300, 200)]);
    }

    #[test]
    fn end_to_end_with_maze_yields_clean_clear_trace() {
        let mut index = ObstacleIndex::new(1);
        index.add_disc(0, Point::new(5000, 5000), 800, 7);
        index.build();
        let bound = IntBox::new(0, 0, 10_000, 10_000);
        let params = MazeParams {
            net: 0, layer: 0, clearance: 0, half_width: 10, bound,
            step: 40, dedup_cell: 40, max_rooms: 5000, window: 0,
        };
        let path = find_path(&index, Point::new(1000, 5000), Point::new(9000, 5000), &params)
            .expect("maze path");
        let line = straighten(&index, &path, 0, 10, 0).unwrap();
        assert_eq!(line.first().unwrap(), &Point::new(1000, 5000));
        assert_eq!(line.last().unwrap(), &Point::new(9000, 5000));
        for w in line.windows(2) {
            assert!(segment_clear(&index, 0, w[0], w[1], 0, 10, 0),
                "final trace segment {:?} must clear copper", w);
        }
    }
}

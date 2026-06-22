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

use fr_geometry::Point;
use fr_spatial::ObstacleIndex;

use crate::maze::PathPoint;

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

    #[test]
    fn end_to_end_with_maze_yields_clean_clear_trace() {
        let mut index = ObstacleIndex::new(1);
        index.add_disc(0, Point::new(5000, 5000), 800, 7);
        index.build();
        let bound = IntBox::new(0, 0, 10_000, 10_000);
        let params = MazeParams {
            net: 0, clearance: 0, half_width: 10, bound,
            step: 40, dedup_cell: 40, max_rooms: 5000,
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

//! Euclidean distance primitives between points and segments.
//!
//! These are the shared geometry kernel for clearance checks: the exact obstacle index
//! (fr-spatial) and the copper-geometry DRC (fr-engine) both reduce "do these two pieces
//! of copper touch?" to a point/segment-to-segment distance compared against the sum of
//! their half-widths. Distances are computed in `f64`; board coordinates are large
//! integers so the relative error is far below one board unit, which is the tolerance the
//! clearance checks already use (they subtract 1 unit of slop).

use crate::point::{Point, Side};

/// Minimum Euclidean distance from point `p` to the closed segment `a`-`b`.
pub fn point_seg_dist(p: Point, a: Point, b: Point) -> f64 {
    let (px, py) = (p.x as f64, p.y as f64);
    let (ax, ay) = (a.x as f64, a.y as f64);
    let (bx, by) = (b.x as f64, b.y as f64);
    let (dx, dy) = (bx - ax, by - ay);
    let len2 = dx * dx + dy * dy;
    if len2 == 0.0 {
        return ((px - ax).powi(2) + (py - ay).powi(2)).sqrt();
    }
    let t = (((px - ax) * dx + (py - ay) * dy) / len2).clamp(0.0, 1.0);
    let (cx, cy) = (ax + t * dx, ay + t * dy);
    ((px - cx).powi(2) + (py - cy).powi(2)).sqrt()
}

/// True if the open segments `p1`-`p2` and `q1`-`q2` cross (proper intersection; shared
/// endpoints or collinear touching do NOT count). Uses exact integer orientation.
pub fn segments_intersect(p1: Point, p2: Point, q1: Point, q2: Point) -> bool {
    let d1 = q1.side_of(q2, p1);
    let d2 = q1.side_of(q2, p2);
    let d3 = p1.side_of(p2, q1);
    let d4 = p1.side_of(p2, q2);
    d1 != d2 && d3 != d4 && d1 != Side::On && d2 != Side::On && d3 != Side::On && d4 != Side::On
}

/// Minimum Euclidean distance between segment `p1`-`p2` and segment `q1`-`q2`.
/// Returns 0.0 if they cross.
pub fn seg_seg_dist(p1: Point, p2: Point, q1: Point, q2: Point) -> f64 {
    if segments_intersect(p1, p2, q1, q2) {
        return 0.0;
    }
    let d = [
        point_seg_dist(p1, q1, q2),
        point_seg_dist(p2, q1, q2),
        point_seg_dist(q1, p1, p2),
        point_seg_dist(q2, p1, p2),
    ];
    d.into_iter().fold(f64::MAX, f64::min)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_to_segment() {
        let a = Point::new(0, 0);
        let b = Point::new(10, 0);
        // foot of perpendicular inside the segment
        assert!((point_seg_dist(Point::new(5, 4), a, b) - 4.0).abs() < 1e-9);
        // beyond an endpoint -> distance to the endpoint
        assert!((point_seg_dist(Point::new(13, 0), a, b) - 3.0).abs() < 1e-9);
        // degenerate segment == point distance
        assert!((point_seg_dist(Point::new(3, 4), a, a) - 5.0).abs() < 1e-9);
    }

    #[test]
    fn crossing_segments_have_zero_distance() {
        let d = seg_seg_dist(
            Point::new(0, 0), Point::new(10, 10),
            Point::new(0, 10), Point::new(10, 0),
        );
        assert_eq!(d, 0.0);
        assert!(segments_intersect(
            Point::new(0, 0), Point::new(10, 10),
            Point::new(0, 10), Point::new(10, 0),
        ));
    }

    #[test]
    fn parallel_segments_distance() {
        // two horizontal segments 5 apart
        let d = seg_seg_dist(
            Point::new(0, 0), Point::new(10, 0),
            Point::new(0, 5), Point::new(10, 5),
        );
        assert!((d - 5.0).abs() < 1e-9);
    }

    #[test]
    fn touching_endpoints_not_a_crossing() {
        // share an endpoint: not a proper crossing, distance 0 via endpoint check
        let a = Point::new(0, 0);
        assert!(!segments_intersect(a, Point::new(10, 0), a, Point::new(0, 10)));
        let d = seg_seg_dist(a, Point::new(10, 0), a, Point::new(0, 10));
        assert!(d.abs() < 1e-9);
    }
}

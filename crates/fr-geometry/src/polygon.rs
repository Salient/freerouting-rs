//! General (possibly concave) polygon point-containment, for the board outline.
//!
//! The board outline can be concave (e.g. an L-shaped board), so the convex `ConvexTile`
//! test doesn't apply. This uses the standard even-odd ray-cast, done in integer/`f64`
//! mix that is robust for board-scale coordinates. Containment is "inside or on".

use crate::point::Point;

/// True if `p` is inside (or on the boundary of) the polygon given by `verts` in order.
/// Even-odd rule. An empty or degenerate (<3 vertex) polygon contains nothing.
pub fn polygon_contains(verts: &[Point], p: Point) -> bool {
    let n = verts.len();
    if n < 3 {
        return false;
    }
    // boundary check first (treat on-edge as inside)
    for i in 0..n {
        let a = verts[i];
        let b = verts[(i + 1) % n];
        if on_segment(a, b, p) {
            return true;
        }
    }
    // even-odd ray cast to +x
    let mut inside = false;
    let (px, py) = (p.x as f64, p.y as f64);
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (verts[i].x as f64, verts[i].y as f64);
        let (xj, yj) = (verts[j].x as f64, verts[j].y as f64);
        let intersect = ((yi > py) != (yj > py))
            && (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
        if intersect {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// Is `p` on the closed segment a-b (collinear and within the bounding box)?
fn on_segment(a: Point, b: Point, p: Point) -> bool {
    // collinear?
    let cross = (b.x - a.x) as i128 * (p.y - a.y) as i128
        - (b.y - a.y) as i128 * (p.x - a.x) as i128;
    if cross != 0 {
        return false;
    }
    p.x >= a.x.min(b.x) && p.x <= a.x.max(b.x) && p.y >= a.y.min(b.y) && p.y <= a.y.max(b.y)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn l_shape() -> Vec<Point> {
        // an L: outer 0..100 square with the top-right quadrant (50..100, 50..100) removed
        vec![
            Point::new(0, 0),
            Point::new(100, 0),
            Point::new(100, 50),
            Point::new(50, 50),
            Point::new(50, 100),
            Point::new(0, 100),
        ]
    }

    #[test]
    fn inside_and_outside_concave() {
        let l = l_shape();
        assert!(polygon_contains(&l, Point::new(25, 25)), "lower-left arm");
        assert!(polygon_contains(&l, Point::new(75, 25)), "lower-right arm");
        assert!(polygon_contains(&l, Point::new(25, 75)), "upper-left arm");
        // the removed notch is outside
        assert!(!polygon_contains(&l, Point::new(75, 75)), "notch is outside");
        assert!(!polygon_contains(&l, Point::new(150, 50)), "far outside");
    }

    #[test]
    fn boundary_is_inside() {
        let l = l_shape();
        assert!(polygon_contains(&l, Point::new(0, 0)), "corner");
        assert!(polygon_contains(&l, Point::new(50, 0)), "edge midpoint");
    }

    #[test]
    fn degenerate_contains_nothing() {
        assert!(!polygon_contains(&[], Point::new(0, 0)));
        assert!(!polygon_contains(&[Point::new(0, 0), Point::new(1, 1)], Point::new(0, 0)));
    }
}

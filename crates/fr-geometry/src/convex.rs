//! Convex polygon (`ConvexTile`), the workhorse shape of the router.
//!
//! The Java model represents a convex region as the intersection of half-planes
//! (`Simplex`). We store the equivalent **ordered CCW vertex list** directly: it is
//! simpler for the operations the router needs (point containment, bounding box,
//! half-plane clipping for lazy room construction) and converts cleanly to/from the
//! half-plane view when needed. All predicates are exact integer arithmetic.

use crate::box2d::IntBox;
use crate::point::{Point, Side};

/// A convex polygon given by vertices in counter-clockwise order. May be empty
/// (no vertices), degenerate (1-2 vertices), or a proper polygon (>= 3 vertices).
/// Invariant for proper polygons: vertices are CCW and the polygon is convex.
#[derive(Clone, PartialEq, Eq, Debug, Default)]
pub struct ConvexTile {
    verts: Vec<Point>,
}

impl ConvexTile {
    pub const fn empty() -> Self {
        ConvexTile { verts: Vec::new() }
    }

    /// Build from vertices assumed already CCW and convex (e.g. from clipping).
    pub fn from_ccw(verts: Vec<Point>) -> Self {
        ConvexTile { verts }
    }

    /// Build an axis-aligned rectangle tile (CCW) from a box.
    pub fn from_box(b: IntBox) -> Self {
        ConvexTile {
            verts: vec![
                Point::new(b.ll.x, b.ll.y),
                Point::new(b.ur.x, b.ll.y),
                Point::new(b.ur.x, b.ur.y),
                Point::new(b.ll.x, b.ur.y),
            ],
        }
    }

    pub fn vertices(&self) -> &[Point] {
        &self.verts
    }

    pub fn is_empty(&self) -> bool {
        self.verts.is_empty()
    }

    /// Number of vertices.
    pub fn len(&self) -> usize {
        self.verts.len()
    }

    /// Axis-aligned bounding box, or None if empty.
    pub fn bounding_box(&self) -> Option<IntBox> {
        IntBox::bound(self.verts.iter().copied())
    }

    /// Twice the signed area (positive for CCW). Exact in i128.
    pub fn signed_area2(&self) -> i128 {
        let n = self.verts.len();
        if n < 3 {
            return 0;
        }
        let mut acc: i128 = 0;
        for i in 0..n {
            let a = self.verts[i];
            let b = self.verts[(i + 1) % n];
            acc += (a.x as i128) * (b.y as i128) - (b.x as i128) * (a.y as i128);
        }
        acc
    }

    /// True if `p` is inside or on the boundary of a proper (>=3 CCW vertices) tile.
    pub fn contains(&self, p: Point) -> bool {
        let n = self.verts.len();
        if n < 3 {
            // degenerate: only the exact vertices "contain" p
            return self.verts.iter().any(|&v| v == p);
        }
        // For a CCW convex polygon, p is inside iff it is not to the Right of any edge.
        for i in 0..n {
            let a = self.verts[i];
            let b = self.verts[(i + 1) % n];
            if a.side_of(b, p) == Side::Right {
                return false;
            }
        }
        true
    }

    /// Number of border edges (== vertex count for a proper polygon).
    pub fn border_line_count(&self) -> usize {
        self.verts.len()
    }

    /// The `i`-th directed border edge as (from, to). CCW, so the interior is on the LEFT
    /// of each edge. Panics only on an empty tile (callers guard with `is_empty`).
    pub fn border_line(&self, i: usize) -> (Point, Point) {
        let n = self.verts.len();
        (self.verts[i % n], self.verts[(i + 1) % n])
    }

    /// Geometric dimension: 2 if it has positive area, 1 if it is a segment/collinear
    /// (>=2 distinct verts but zero area), 0 if a single point, -1 if empty.
    pub fn dimension(&self) -> i32 {
        match self.verts.len() {
            0 => -1,
            1 => 0,
            _ => {
                if self.signed_area2() != 0 {
                    2
                } else {
                    // distinct points but no area -> segment (1) or coincident point (0)
                    let first = self.verts[0];
                    if self.verts.iter().all(|&v| v == first) { 0 } else { 1 }
                }
            }
        }
    }

    /// Intersection of two convex tiles (Sutherland–Hodgman: clip `self` by every edge
    /// half-plane of `other`). Both must be CCW. Returns a possibly-empty/degenerate tile.
    pub fn intersection(&self, other: &ConvexTile) -> ConvexTile {
        if self.is_empty() || other.is_empty() {
            return ConvexTile::empty();
        }
        let mut out = self.clone();
        let n = other.verts.len();
        for i in 0..n {
            if out.is_empty() {
                break;
            }
            let (a, b) = (other.verts[i], other.verts[(i + 1) % n]);
            out = out.clip_halfplane(a, b);
        }
        out
    }

    /// True if the directed line through a->b passes through the interior of this tile,
    /// i.e. the tile has corners strictly on both sides of the line. (Mirrors the Java
    /// `TileShape.side_of(line) == COLLINEAR` test used by room restraining.)
    pub fn line_intersects_interior(&self, a: Point, b: Point) -> bool {
        let mut left = false;
        let mut right = false;
        for &c in &self.verts {
            match a.side_of(b, c) {
                Side::Left => left = true,
                Side::Right => right = true,
                Side::On => {}
            }
            if left && right {
                return true;
            }
        }
        false
    }

    /// Clip this convex tile to the closed left half-plane of the directed line a->b
    /// (keep the part that is Left or On). Returns a new convex tile (possibly empty).
    /// This is the primitive used to carve obstacle-free rooms out of free space.
    pub fn clip_halfplane(&self, a: Point, b: Point) -> ConvexTile {
        let n = self.verts.len();
        if n == 0 {
            return ConvexTile::empty();
        }
        let mut out: Vec<Point> = Vec::with_capacity(n + 1);
        for i in 0..n {
            let cur = self.verts[i];
            let nxt = self.verts[(i + 1) % n];
            let cur_in = a.side_of(b, cur) != Side::Right;
            let nxt_in = a.side_of(b, nxt) != Side::Right;
            if cur_in {
                out.push(cur);
            }
            if cur_in != nxt_in {
                // edge crosses the line: add the intersection point (rounded to int)
                if let Some(ip) = line_segment_intersection(a, b, cur, nxt) {
                    // avoid duplicating an endpoint that is exactly On the line
                    if out.last() != Some(&ip) {
                        out.push(ip);
                    }
                }
            }
        }
        // de-duplicate consecutive equal points
        out.dedup();
        if out.len() >= 2 && out.first() == out.last() {
            out.pop();
        }
        ConvexTile::from_ccw(out)
    }
}

/// Intersection point of infinite line a->b with segment [c,d], rounded to the nearest
/// integer point. Returns None if parallel. Used by half-plane clipping.
fn line_segment_intersection(a: Point, b: Point, c: Point, d: Point) -> Option<Point> {
    // line a->b direction; param t on segment c->d where it crosses.
    // Solve cross((d-c), (b-a)) etc. using f64 for the ratio, then round (coords are
    // large integers, so rounding error is sub-unit and acceptable for board geometry).
    let r = a.diff(b); // b - a
    let s = c.diff(d); // d - c
    let denom = (r.x as f64) * (s.y as f64) - (r.y as f64) * (s.x as f64);
    if denom == 0.0 {
        return None;
    }
    // t such that point = c + t*(d-c) lies on line a->b
    let acx = (c.x - a.x) as f64;
    let acy = (c.y - a.y) as f64;
    let t = (acx * (r.y as f64) - acy * (r.x as f64)) / denom;
    let px = c.x as f64 + t * (s.x as f64);
    let py = c.y as f64 + t * (s.y as f64);
    Some(Point::new(px.round() as i64, py.round() as i64))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_square() -> ConvexTile {
        ConvexTile::from_box(IntBox::new(0, 0, 100, 100))
    }

    #[test]
    fn box_is_ccw_and_has_positive_area() {
        let t = unit_square();
        assert_eq!(t.len(), 4);
        assert!(t.signed_area2() > 0, "from_box must be CCW (positive area)");
        assert_eq!(t.signed_area2(), 2 * 100 * 100);
    }

    #[test]
    fn contains_inside_boundary_outside() {
        let t = unit_square();
        assert!(t.contains(Point::new(50, 50)));
        assert!(t.contains(Point::new(0, 0)));   // corner
        assert!(t.contains(Point::new(100, 50))); // edge
        assert!(!t.contains(Point::new(101, 50)));
        assert!(!t.contains(Point::new(-1, -1)));
    }

    #[test]
    fn clip_halfplane_cuts_square_in_half() {
        let t = unit_square();
        // clip to left of the vertical line x=50 going upward (a=(50,0)->b=(50,100)):
        // left side of an upward line is x<50.
        let clipped = t.clip_halfplane(Point::new(50, 0), Point::new(50, 100));
        let bb = clipped.bounding_box().unwrap();
        assert_eq!(bb.ll.x, 0);
        assert_eq!(bb.ur.x, 50, "kept the x<=50 half");
        assert!(clipped.contains(Point::new(25, 50)));
        assert!(!clipped.contains(Point::new(75, 50)));
    }

    #[test]
    fn clip_fully_outside_yields_empty() {
        let t = unit_square();
        // Upward vertical line at x=-10: its Left half-plane is x < -10, which excludes
        // the whole square (x in 0..100), so the clip result has no area.
        let clipped = t.clip_halfplane(Point::new(-10, 0), Point::new(-10, 100));
        assert!(clipped.is_empty() || clipped.signed_area2() == 0);
    }

    #[test]
    fn clip_fully_inside_keeps_all() {
        let t = unit_square();
        // Downward line at x=-10: Left half-plane is x > -10, which contains the square,
        // so the clip keeps the whole tile.
        let clipped = t.clip_halfplane(Point::new(-10, 100), Point::new(-10, 0));
        assert_eq!(clipped.signed_area2(), t.signed_area2());
    }

    #[test]
    fn dimension_classification() {
        assert_eq!(ConvexTile::empty().dimension(), -1);
        assert_eq!(ConvexTile::from_ccw(vec![Point::new(5, 5)]).dimension(), 0);
        assert_eq!(ConvexTile::from_ccw(vec![Point::new(0, 0), Point::new(10, 0)]).dimension(), 1);
        assert_eq!(unit_square().dimension(), 2);
    }

    #[test]
    fn intersection_of_overlapping_squares() {
        let a = ConvexTile::from_box(IntBox::new(0, 0, 100, 100));
        let b = ConvexTile::from_box(IntBox::new(50, 50, 200, 200));
        let i = a.intersection(&b);
        let bb = i.bounding_box().unwrap();
        assert_eq!(bb, IntBox::new(50, 50, 100, 100));
        assert_eq!(i.dimension(), 2);
        assert!(i.contains(Point::new(75, 75)));
        assert!(!i.contains(Point::new(150, 150)));
    }

    #[test]
    fn intersection_disjoint_is_empty_or_degenerate() {
        let a = ConvexTile::from_box(IntBox::new(0, 0, 10, 10));
        let b = ConvexTile::from_box(IntBox::new(100, 100, 110, 110));
        let i = a.intersection(&b);
        assert!(i.dimension() < 2, "disjoint tiles do not overlap in area");
    }

    #[test]
    fn line_intersects_interior_basic() {
        let sq = unit_square(); // 0..100
        // vertical line through the middle (x=50) crosses the interior
        assert!(sq.line_intersects_interior(Point::new(50, -10), Point::new(50, 110)));
        // a line entirely to the left (x=-5) does not
        assert!(!sq.line_intersects_interior(Point::new(-5, -10), Point::new(-5, 110)));
        // a line along the right edge (x=100) only touches the border, not the interior
        assert!(!sq.line_intersects_interior(Point::new(100, -10), Point::new(100, 110)));
    }

    #[test]
    fn border_lines_are_ccw() {
        let sq = unit_square();
        assert_eq!(sq.border_line_count(), 4);
        // the interior point (50,50) is left-of every CCW border edge
        for i in 0..sq.border_line_count() {
            let (a, b) = sq.border_line(i);
            assert_ne!(a.side_of(b, Point::new(50, 50)), Side::Right);
        }
    }
}

//! Axis-aligned integer bounding box (`IntBox` in the Java model).
//!
//! Used everywhere as a cheap conservative bound: R-tree keys, quick reject tests
//! before exact shape intersection, and the per-layer destination boxes the routing
//! heuristic measures against.

use crate::point::Point;
use crate::Coord;

/// An axis-aligned bounding box with inclusive integer bounds. `ll` is the lower-left
/// (min) corner, `ur` the upper-right (max) corner. Empty if `ll > ur` on either axis.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct IntBox {
    pub ll: Point,
    pub ur: Point,
}

impl IntBox {
    pub const fn new(min_x: Coord, min_y: Coord, max_x: Coord, max_y: Coord) -> Self {
        IntBox { ll: Point::new(min_x, min_y), ur: Point::new(max_x, max_y) }
    }

    /// The smallest box containing both points.
    pub fn from_points(a: Point, b: Point) -> Self {
        IntBox {
            ll: Point::new(a.x.min(b.x), a.y.min(b.y)),
            ur: Point::new(a.x.max(b.x), a.y.max(b.y)),
        }
    }

    /// Bounding box of a set of points. Returns None for an empty iterator.
    pub fn bound<I: IntoIterator<Item = Point>>(points: I) -> Option<IntBox> {
        let mut it = points.into_iter();
        let first = it.next()?;
        let mut b = IntBox { ll: first, ur: first };
        for p in it {
            b = b.extend(p);
        }
        Some(b)
    }

    pub fn is_empty(&self) -> bool {
        self.ll.x > self.ur.x || self.ll.y > self.ur.y
    }

    pub fn width(&self) -> Coord {
        self.ur.x - self.ll.x
    }

    pub fn height(&self) -> Coord {
        self.ur.y - self.ll.y
    }

    pub fn contains(&self, p: Point) -> bool {
        p.x >= self.ll.x && p.x <= self.ur.x && p.y >= self.ll.y && p.y <= self.ur.y
    }

    /// Grow the box to include `p`.
    pub fn extend(self, p: Point) -> IntBox {
        IntBox {
            ll: Point::new(self.ll.x.min(p.x), self.ll.y.min(p.y)),
            ur: Point::new(self.ur.x.max(p.x), self.ur.y.max(p.y)),
        }
    }

    /// Union with another box.
    pub fn union(self, other: IntBox) -> IntBox {
        IntBox {
            ll: Point::new(self.ll.x.min(other.ll.x), self.ll.y.min(other.ll.y)),
            ur: Point::new(self.ur.x.max(other.ur.x), self.ur.y.max(other.ur.y)),
        }
    }

    /// True if the two boxes share any area or boundary (closed overlap).
    pub fn intersects(&self, other: &IntBox) -> bool {
        self.ll.x <= other.ur.x
            && self.ur.x >= other.ll.x
            && self.ll.y <= other.ur.y
            && self.ur.y >= other.ll.y
    }

    /// Expand outward by `d` on every side (negative shrinks).
    pub fn offset(self, d: Coord) -> IntBox {
        IntBox {
            ll: Point::new(self.ll.x - d, self.ll.y - d),
            ur: Point::new(self.ur.x + d, self.ur.y + d),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bound_and_contains() {
        let b = IntBox::bound([
            Point::new(1, 2),
            Point::new(5, -3),
            Point::new(-2, 4),
        ])
        .unwrap();
        assert_eq!(b, IntBox::new(-2, -3, 5, 4));
        assert!(b.contains(Point::new(0, 0)));
        assert!(!b.contains(Point::new(6, 0)));
    }

    #[test]
    fn intersects_and_union() {
        let a = IntBox::new(0, 0, 10, 10);
        let b = IntBox::new(10, 10, 20, 20); // touches at a corner
        assert!(a.intersects(&b));
        let c = IntBox::new(11, 11, 20, 20);
        assert!(!a.intersects(&c));
        assert_eq!(a.union(c), IntBox::new(0, 0, 20, 20));
    }

    #[test]
    fn offset_grows_and_shrinks() {
        let a = IntBox::new(0, 0, 10, 10);
        assert_eq!(a.offset(5), IntBox::new(-5, -5, 15, 15));
        assert_eq!(a.offset(-2), IntBox::new(2, 2, 8, 8));
    }
}

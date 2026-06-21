//! Integer and floating-point points, vectors, and the core orientation predicate.
//!
//! Ported in spirit from the Java `geometry.planar` package (IntPoint, IntVector,
//! FloatPoint). Board coordinates are exact `i64`; cross products accumulate in `i128`
//! so we never overflow on realistic board extents (Specctra coords are well within
//! +/- 2^31 even at 1e4 units/mil, and i128 products are exact).

use crate::Coord;

/// An exact integer point in board units.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct Point {
    pub x: Coord,
    pub y: Coord,
}

/// An exact integer vector (difference of two points).
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct Vector {
    pub x: Coord,
    pub y: Coord,
}

/// A floating-point point, used for approximations, output rounding, and length math.
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct FloatPoint {
    pub x: f64,
    pub y: f64,
}

/// Orientation of point `c` relative to the directed line `a -> b`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Side {
    /// c is to the left of a->b (counter-clockwise turn).
    Left,
    /// c is exactly on the line a->b.
    On,
    /// c is to the right of a->b (clockwise turn).
    Right,
}

impl Point {
    pub const fn new(x: Coord, y: Coord) -> Self {
        Point { x, y }
    }

    /// Vector from `self` to `other` (other - self).
    pub fn diff(self, other: Point) -> Vector {
        Vector { x: other.x - self.x, y: other.y - self.y }
    }

    /// Translate by a vector.
    pub fn translate(self, v: Vector) -> Point {
        Point { x: self.x + v.x, y: self.y + v.y }
    }

    pub fn to_float(self) -> FloatPoint {
        FloatPoint { x: self.x as f64, y: self.y as f64 }
    }

    /// Squared Euclidean distance to `other` (exact, in `i128` to avoid overflow).
    pub fn distance_square(self, other: Point) -> i128 {
        let dx = (other.x - self.x) as i128;
        let dy = (other.y - self.y) as i128;
        dx * dx + dy * dy
    }

    /// Orientation of `c` relative to the directed line `self -> b`.
    ///
    /// Uses the sign of the 2D cross product (b-self) x (c-self), computed exactly in
    /// `i128`. Returns Left for a counter-clockwise turn, Right for clockwise, On if
    /// the three points are collinear.
    pub fn side_of(self, b: Point, c: Point) -> Side {
        let cross = signed_area2(self, b, c);
        if cross > 0 {
            Side::Left
        } else if cross < 0 {
            Side::Right
        } else {
            Side::On
        }
    }
}

/// Twice the signed area of triangle (a, b, c): (b-a) x (c-a). Positive = CCW.
/// Exact in i128.
pub fn signed_area2(a: Point, b: Point, c: Point) -> i128 {
    let abx = (b.x - a.x) as i128;
    let aby = (b.y - a.y) as i128;
    let acx = (c.x - a.x) as i128;
    let acy = (c.y - a.y) as i128;
    abx * acy - aby * acx
}

impl Vector {
    pub const fn new(x: Coord, y: Coord) -> Self {
        Vector { x, y }
    }

    /// Exact 2D cross product self x other (z component), in i128.
    pub fn cross(self, other: Vector) -> i128 {
        (self.x as i128) * (other.y as i128) - (self.y as i128) * (other.x as i128)
    }

    /// Exact dot product, in i128.
    pub fn dot(self, other: Vector) -> i128 {
        (self.x as i128) * (other.x as i128) + (self.y as i128) * (other.y as i128)
    }

    pub fn add(self, other: Vector) -> Vector {
        Vector { x: self.x + other.x, y: self.y + other.y }
    }

    pub fn is_zero(self) -> bool {
        self.x == 0 && self.y == 0
    }
}

impl FloatPoint {
    pub fn new(x: f64, y: f64) -> Self {
        FloatPoint { x, y }
    }

    pub fn distance(self, other: FloatPoint) -> f64 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        (dx * dx + dy * dy).sqrt()
    }

    /// Round to the nearest integer point.
    pub fn round(self) -> Point {
        Point { x: self.x.round() as Coord, y: self.y.round() as Coord }
    }

    /// Midpoint with another float point.
    pub fn middle(self, other: FloatPoint) -> FloatPoint {
        FloatPoint { x: 0.5 * (self.x + other.x), y: 0.5 * (self.y + other.y) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orientation_basic() {
        let a = Point::new(0, 0);
        let b = Point::new(10, 0);
        assert_eq!(a.side_of(b, Point::new(5, 5)), Side::Left);
        assert_eq!(a.side_of(b, Point::new(5, -5)), Side::Right);
        assert_eq!(a.side_of(b, Point::new(5, 0)), Side::On);
    }

    #[test]
    fn orientation_no_overflow_large_coords() {
        // Coordinates near Specctra scale (mil * 1e4) on a large board.
        let a = Point::new(60_000_000, 80_000_000);
        let b = Point::new(190_000_000, 80_000_000);
        // A point clearly above the line must be Left regardless of magnitude.
        assert_eq!(a.side_of(b, Point::new(120_000_000, 155_000_000)), Side::Left);
    }

    #[test]
    fn distance_square_exact() {
        let a = Point::new(0, 0);
        let b = Point::new(3, 4);
        assert_eq!(a.distance_square(b), 25);
    }

    #[test]
    fn cross_and_dot() {
        let u = Vector::new(1, 0);
        let v = Vector::new(0, 1);
        assert_eq!(u.cross(v), 1);
        assert_eq!(u.dot(v), 0);
        assert_eq!(u.dot(u), 1);
    }
}

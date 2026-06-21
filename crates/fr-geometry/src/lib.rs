//! fr-geometry: fixed-point integer geometry primitives for freerouting-rs.
//!
//! Coordinates are `i64` "board units" (the Specctra resolution unit, e.g. 1 mil =
//! 10000 units for an Altium `(resolution mil 10000)` design). Integer arithmetic is
//! exact and reproducible, which the parallel router relies on for determinism.
//! Cross products and areas accumulate in `i128` so they never overflow on realistic
//! board extents.
//!
//! Phase 1 (freerouting-rs-spec/MILESTONES.md): points, vectors, the orientation
//! predicate, and axis-aligned boxes. Convex tiles (Simplex), polylines and areas
//! build on these next.

mod box2d;
mod point;

pub use box2d::IntBox;
pub use point::{signed_area2, FloatPoint, Point, Side, Vector};

/// A board coordinate in integer board units.
pub type Coord = i64;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reexports_usable() {
        let a = Point::new(0, 0);
        let b = Point::new(10, 10);
        let bb = IntBox::from_points(a, b);
        assert!(bb.contains(Point::new(5, 5)));
        assert_eq!(a.side_of(Point::new(10, 0), b), Side::Left);
    }
}

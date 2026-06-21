//! fr-geometry: fixed-point integer geometry primitives for freerouting-rs.
//!
//! Coordinates are `i64` "board units" (the Specctra resolution unit, e.g. 1 mil =
//! 10000 units for an Altium `(resolution mil 10000)` design). Integer arithmetic is
//! exact and reproducible, which the parallel router relies on for determinism.
//!
//! Phase 1 (see freerouting-rs-spec/MILESTONES.md) fills this in with points, vectors,
//! convex tiles (Simplex), polylines, areas, and the orientation/intersection
//! predicates. For now it carries only the coordinate type and a placeholder.

/// A board coordinate in integer board units.
pub type Coord = i64;

#[cfg(test)]
mod tests {
    #[test]
    fn crate_builds() {
        // Phase 0 gate: the crate compiles and a trivial test runs.
        assert_eq!(2 + 2, 4);
    }
}

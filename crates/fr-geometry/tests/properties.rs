//! Property tests for fr-geometry (Phase 1 gate). Uses proptest to check geometric
//! invariants hold across randomized inputs.

use fr_geometry::{ConvexTile, IntBox, Point, Side};
use proptest::prelude::*;

// Keep coordinates large (Specctra-scale) but bounded so products stay well within i128.
fn coord() -> impl Strategy<Value = i64> {
    -1_000_000_000i64..1_000_000_000i64
}

fn pt() -> impl Strategy<Value = Point> {
    (coord(), coord()).prop_map(|(x, y)| Point::new(x, y))
}

fn intbox() -> impl Strategy<Value = IntBox> {
    (coord(), coord(), 0i64..2_000_000i64, 0i64..2_000_000i64)
        .prop_map(|(x, y, w, h)| IntBox::new(x, y, x + w, y + h))
}

proptest! {
    /// A box tile is always CCW (non-negative signed area) and convex.
    #[test]
    fn box_tile_is_ccw(b in intbox()) {
        let t = ConvexTile::from_box(b);
        prop_assert!(t.signed_area2() >= 0);
    }

    /// Clipping a convex tile to a half-plane never increases its area (it removes a
    /// region) and never makes area negative (stays CCW / convex).
    #[test]
    fn clip_does_not_grow_or_flip(b in intbox(), a in pt(), c in pt()) {
        prop_assume!(a != c); // need a real directed line
        let t = ConvexTile::from_box(b);
        let orig = t.signed_area2();
        let clipped = t.clip_halfplane(a, c);
        let new = clipped.signed_area2();
        prop_assert!(new >= 0, "clipped area must stay non-negative (CCW)");
        // allow a tiny slack for integer-rounding of intersection points
        prop_assert!(new <= orig + 4, "clip must not grow the tile (area {new} > {orig})");
    }

    /// Every vertex of a clipped tile is on the kept (Left or On) side of the cut line.
    #[test]
    fn clipped_vertices_on_kept_side(b in intbox(), a in pt(), c in pt()) {
        prop_assume!(a != c);
        let t = ConvexTile::from_box(b);
        let clipped = t.clip_halfplane(a, c);
        for &v in clipped.vertices() {
            // rounding of intersection points can place a vertex up to ~1 unit on the
            // wrong side; assert it is not grossly outside by re-checking with slack via
            // the signed area magnitude relative to the line.
            let side = a.side_of(c, v);
            // On or Left is exact-kept; Right only acceptable as a rounding artifact,
            // which we bound by requiring it be extremely close to the line.
            if side == Side::Right {
                let area = fr_geometry::signed_area2(a, c, v).abs();
                let line_len_sq = a.distance_square(c).max(1);
                // perpendicular distance^2 ~= (2*area)^2 / line_len_sq must be < ~4 units^2
                prop_assert!((4 * area * area) <= 16 * line_len_sq,
                    "vertex too far on the wrong side after clip");
            }
        }
    }

    /// Orientation is anti-symmetric: side_of(a,b,c) is the mirror of side_of(b,a,c).
    #[test]
    fn orientation_antisymmetric(a in pt(), b in pt(), c in pt()) {
        prop_assume!(a != b);
        let s1 = a.side_of(b, c);
        let s2 = b.side_of(a, c);
        let expected = match s1 {
            Side::Left => Side::Right,
            Side::Right => Side::Left,
            Side::On => Side::On,
        };
        prop_assert_eq!(s2, expected);
    }

    /// IntBox::bound contains every point it was built from.
    #[test]
    fn bound_contains_inputs(pts in proptest::collection::vec(pt(), 1..20)) {
        let bb = IntBox::bound(pts.iter().copied()).unwrap();
        for &p in &pts {
            prop_assert!(bb.contains(p));
        }
    }

    /// from_box round-trips through bounding_box.
    #[test]
    fn box_tile_bbox_roundtrip(b in intbox()) {
        let t = ConvexTile::from_box(b);
        prop_assert_eq!(t.bounding_box().unwrap(), b);
    }
}

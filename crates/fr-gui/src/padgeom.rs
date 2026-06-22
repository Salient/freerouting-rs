//! Shared pad geometry: turn a pin's padstack into a drawable shape in absolute board
//! coordinates, so the egui canvas (app.rs) and the software renderer (render.rs) draw
//! identical pads. The model stores per-layer shapes as `Circle{radius}` (centered on the
//! pad origin) or `Convex(tile)` (vertices relative to the pad origin); we pick the
//! largest copper shape across the padstack's layers as the pad's visible footprint.

use fr_board::{Board, PadShape, Pin};
use fr_geometry::Point;

/// A drawable pad footprint in absolute board units.
#[derive(Clone, Debug)]
pub enum PadDraw {
    /// Filled circle of `radius` board units at `center`.
    Circle { center: Point, radius: i64 },
    /// Filled convex polygon (absolute board-unit vertices, CCW).
    Poly(Vec<Point>),
}

impl PadDraw {
    /// A rough "size" used to pick the largest shape across layers (radius for a circle,
    /// half the bbox diagonal for a polygon).
    fn extent(&self) -> i64 {
        match self {
            PadDraw::Circle { radius, .. } => *radius,
            PadDraw::Poly(v) => fr_geometry::IntBox::bound(v.iter().copied())
                .map(|b| (b.width() + b.height()) / 2)
                .unwrap_or(0),
        }
    }
}

/// The visible pad footprint of `pin` (its largest copper shape across all layers),
/// translated to absolute board coordinates. Returns None for a shapeless padstack.
pub fn pin_pad_shape(board: &Board, pin: &Pin) -> Option<PadDraw> {
    let ps = board.padstacks.get(pin.padstack)?;
    if ps.is_empty() {
        return None;
    }
    let mut best: Option<PadDraw> = None;
    for shape in ps.shapes.iter().flatten() {
        let draw = match shape {
            PadShape::Circle { radius } => PadDraw::Circle { center: pin.location, radius: *radius },
            PadShape::Convex(tile) => {
                let verts: Vec<Point> = tile
                    .vertices()
                    .iter()
                    .map(|&v| Point::new(pin.location.x + v.x, pin.location.y + v.y))
                    .collect();
                if verts.len() < 3 {
                    continue;
                }
                PadDraw::Poly(verts)
            }
        };
        if best.as_ref().map(|b| draw.extent() > b.extent()).unwrap_or(true) {
            best = Some(draw);
        }
    }
    best
}

/// Ear-clipping triangulation of a simple polygon (possibly concave, e.g. an L-shaped
/// board outline). Returns a flat list of triangles as index triples into `verts`.
/// Robust enough for board outlines (no holes, no self-intersection). Empty if the
/// polygon is degenerate (<3 vertices).
pub fn triangulate(verts: &[Point]) -> Vec<[usize; 3]> {
    let n = verts.len();
    if n < 3 {
        return Vec::new();
    }
    // signed area to determine winding; we operate on a CCW copy of the index ring.
    let area2: i128 = (0..n)
        .map(|i| {
            let a = verts[i];
            let b = verts[(i + 1) % n];
            (a.x as i128) * (b.y as i128) - (b.x as i128) * (a.y as i128)
        })
        .sum();
    let mut idx: Vec<usize> = (0..n).collect();
    if area2 < 0 {
        idx.reverse(); // make CCW
    }

    let mut tris = Vec::with_capacity(n.saturating_sub(2));
    let mut guard = 0usize;
    while idx.len() > 3 {
        let m = idx.len();
        let mut clipped = false;
        for i in 0..m {
            let ia = idx[(i + m - 1) % m];
            let ib = idx[i];
            let ic = idx[(i + 1) % m];
            let (a, b, c) = (verts[ia], verts[ib], verts[ic]);
            // convex corner? (CCW => left turn, cross > 0)
            let cross = (b.x - a.x) as i128 * (c.y - a.y) as i128
                - (b.y - a.y) as i128 * (c.x - a.x) as i128;
            if cross <= 0 {
                continue; // reflex or collinear: not an ear tip
            }
            // no other vertex inside triangle a-b-c?
            let mut empty = true;
            for &j in &idx {
                if j == ia || j == ib || j == ic {
                    continue;
                }
                if point_in_tri(verts[j], a, b, c) {
                    empty = false;
                    break;
                }
            }
            if empty {
                tris.push([ia, ib, ic]);
                idx.remove(i);
                clipped = true;
                break;
            }
        }
        guard += 1;
        if !clipped || guard > n * n + 4 {
            break; // degenerate / numerical fallback: stop rather than loop forever
        }
    }
    if idx.len() == 3 {
        tris.push([idx[0], idx[1], idx[2]]);
    }
    tris
}

/// Point-in-triangle (inclusive), exact integer.
fn point_in_tri(p: Point, a: Point, b: Point, c: Point) -> bool {
    let d = |u: Point, v: Point, w: Point| -> i128 {
        (v.x - u.x) as i128 * (w.y - u.y) as i128 - (v.y - u.y) as i128 * (w.x - u.x) as i128
    };
    let d1 = d(a, b, p);
    let d2 = d(b, c, p);
    let d3 = d(c, a, p);
    let has_neg = d1 < 0 || d2 < 0 || d3 < 0;
    let has_pos = d1 > 0 || d2 > 0 || d3 > 0;
    !(has_neg && has_pos)
}

#[cfg(test)]
mod tests {
    use super::*;
    use fr_board::{Padstack, Resolution, Unit};
    use fr_geometry::ConvexTile;

    fn board_with_pin(shapes: Vec<Option<PadShape>>) -> (Board, Pin) {
        let mut b = Board::new("t".into(), Resolution::new(Unit::Mil, 10000));
        let idx = b.padstacks.add(Padstack { name: "P".into(), shapes, drillable: false });
        let pin = Pin {
            component: "U1".into(),
            name: "1".into(),
            padstack: idx,
            location: Point::new(1000, 2000),
            net: Some(0),
        };
        (b, pin)
    }

    #[test]
    fn circle_pad_is_absolute() {
        let (b, pin) = board_with_pin(vec![Some(PadShape::Circle { radius: 50 })]);
        match pin_pad_shape(&b, &pin).unwrap() {
            PadDraw::Circle { center, radius } => {
                assert_eq!(center, Point::new(1000, 2000));
                assert_eq!(radius, 50);
            }
            _ => panic!("expected circle"),
        }
    }

    #[test]
    fn picks_largest_shape_across_layers() {
        let (b, pin) = board_with_pin(vec![
            Some(PadShape::Circle { radius: 20 }),
            Some(PadShape::Circle { radius: 80 }),
        ]);
        match pin_pad_shape(&b, &pin).unwrap() {
            PadDraw::Circle { radius, .. } => assert_eq!(radius, 80),
            _ => panic!("expected circle"),
        }
    }

    #[test]
    fn convex_pad_translated_to_absolute() {
        let tile = ConvexTile::from_ccw(vec![
            Point::new(-10, -10), Point::new(10, -10), Point::new(10, 10), Point::new(-10, 10),
        ]);
        let (b, pin) = board_with_pin(vec![Some(PadShape::Convex(tile))]);
        match pin_pad_shape(&b, &pin).unwrap() {
            PadDraw::Poly(v) => {
                assert_eq!(v.len(), 4);
                assert!(v.contains(&Point::new(990, 1990)));
                assert!(v.contains(&Point::new(1010, 2010)));
            }
            _ => panic!("expected poly"),
        }
    }

    #[test]
    fn shapeless_padstack_has_no_shape() {
        let (b, pin) = board_with_pin(vec![None, None]);
        assert!(pin_pad_shape(&b, &pin).is_none());
    }

    #[test]
    fn triangulate_square_two_triangles() {
        let sq = vec![Point::new(0, 0), Point::new(10, 0), Point::new(10, 10), Point::new(0, 10)];
        let tris = triangulate(&sq);
        assert_eq!(tris.len(), 2, "a quad triangulates into 2 triangles");
    }

    #[test]
    fn triangulate_l_shape_covers_arms() {
        // L-shape (concave): 6 vertices -> 4 triangles
        let l = vec![
            Point::new(0, 0), Point::new(100, 0), Point::new(100, 50),
            Point::new(50, 50), Point::new(50, 100), Point::new(0, 100),
        ];
        let tris = triangulate(&l);
        assert_eq!(tris.len(), 4, "n-2 triangles for a simple polygon");
        // a point in the notch (75,75) must NOT be covered by any triangle
        let notch = Point::new(75, 75);
        let covered = tris.iter().any(|t| point_in_tri(notch, l[t[0]], l[t[1]], l[t[2]]));
        assert!(!covered, "the L notch must remain uncovered");
        // a point in the lower-right arm (75,25) MUST be covered
        let arm = Point::new(75, 25);
        let covered = tris.iter().any(|t| point_in_tri(arm, l[t[0]], l[t[1]], l[t[2]]));
        assert!(covered, "the L arm must be filled");
    }

    #[test]
    fn triangulate_handles_clockwise_input() {
        // same square but CW: must still produce 2 triangles
        let cw = vec![Point::new(0, 0), Point::new(0, 10), Point::new(10, 10), Point::new(10, 0)];
        assert_eq!(triangulate(&cw).len(), 2);
    }

    #[test]
    fn triangulate_degenerate() {
        assert!(triangulate(&[]).is_empty());
        assert!(triangulate(&[Point::new(0, 0), Point::new(1, 1)]).is_empty());
    }
}

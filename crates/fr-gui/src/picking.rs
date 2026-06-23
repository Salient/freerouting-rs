//! Hit-testing: find the board item (trace segment, via, or pad) nearest a board-space
//! query point, within a tolerance. Used for hover tooltips and click selection. All
//! geometry is in board units; the GUI converts the cursor's pixel tolerance into board
//! units via the view scale before calling in.

use fr_board::Board;
use fr_geometry::{point_seg_dist, Point};

/// A selectable board item, identified by its kind and index into the board's vectors.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Pick {
    Trace { index: usize },
    Via { index: usize },
    Pad { pin_index: usize },
}

/// Find the item closest to `q` whose copper is within `tol` board units of `q`.
/// Priority on ties (smallest distance wins; equal distance prefers pads > vias > traces,
/// since pads/vias are the click targets a user usually wants). `layer_visible` gates
/// traces by layer; pads/vias are layer-spanning and always considered.
pub fn pick_at(board: &Board, q: Point, tol: f64, layer_visible: &[bool]) -> Option<Pick> {
    let mut best: Option<(f64, u8, Pick)> = None;
    let consider = |dist: f64, prio: u8, pick: Pick, best: &mut Option<(f64, u8, Pick)>| {
        if dist > tol {
            return;
        }
        let better = match best {
            None => true,
            Some((bd, bp, _)) => dist < *bd - 1e-9 || (((dist - *bd).abs() <= 1e-9) && prio > *bp),
        };
        if better {
            *best = Some((dist, prio, pick));
        }
    };

    // pads (priority 2)
    for (i, pin) in board.pins.iter().enumerate() {
        if let Some(shape) = crate::padgeom::pin_pad_shape(board, pin) {
            let d = pad_distance(&shape, q);
            consider(d, 2, Pick::Pad { pin_index: i }, &mut best);
        }
    }
    // vias (priority 1) — treat as a disc of the via padstack's max circle radius
    for (i, v) in board.vias.iter().enumerate() {
        let r = via_radius(board, v.padstack) as f64;
        let d = (dist(q, v.location) - r).max(0.0);
        consider(d, 1, Pick::Via { index: i }, &mut best);
    }
    // traces (priority 0) — distance to the polyline minus half width
    for (i, t) in board.traces.iter().enumerate() {
        if t.layer < layer_visible.len() && !layer_visible[t.layer] {
            continue;
        }
        let half = (t.width as f64) / 2.0;
        let mut dmin = f64::MAX;
        for seg in t.corners.windows(2) {
            dmin = dmin.min(point_seg_dist(q, seg[0], seg[1]));
        }
        consider((dmin - half).max(0.0), 0, Pick::Trace { index: i }, &mut best);
    }

    best.map(|(_, _, p)| p)
}

/// Max circle radius across a via padstack's layers (0 if none).
fn via_radius(board: &Board, padstack: usize) -> i64 {
    board
        .padstacks
        .get(padstack)
        .map(|p| {
            p.shapes
                .iter()
                .filter_map(|s| match s {
                    Some(fr_board::PadShape::Circle { radius }) => Some(*radius),
                    _ => None,
                })
                .max()
                .unwrap_or(0)
        })
        .unwrap_or(0)
}

fn dist(a: Point, b: Point) -> f64 {
    let dx = (a.x - b.x) as f64;
    let dy = (a.y - b.y) as f64;
    (dx * dx + dy * dy).sqrt()
}

/// Distance from `q` to the copper of a pad shape (0 if inside).
fn pad_distance(shape: &crate::padgeom::PadDraw, q: Point) -> f64 {
    match shape {
        crate::padgeom::PadDraw::Circle { center, radius } => {
            (dist(q, *center) - *radius as f64).max(0.0)
        }
        crate::padgeom::PadDraw::Poly(verts) => {
            if point_in_poly(verts, q) {
                0.0
            } else {
                // distance to the nearest edge
                let mut dmin = f64::MAX;
                for i in 0..verts.len() {
                    let a = verts[i];
                    let b = verts[(i + 1) % verts.len()];
                    dmin = dmin.min(point_seg_dist(q, a, b));
                }
                dmin
            }
        }
    }
}

/// Even-odd point-in-polygon for a simple polygon (board units).
fn point_in_poly(verts: &[Point], p: Point) -> bool {
    let n = verts.len();
    if n < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (verts[i].x as f64, verts[i].y as f64);
        let (xj, yj) = (verts[j].x as f64, verts[j].y as f64);
        let (px, py) = (p.x as f64, p.y as f64);
        if ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

#[cfg(test)]
mod tests {
    use super::*;
    use fr_board::{FixedState, Layer, LayerStack, PadShape, Padstack, Pin, Resolution, Trace, Unit, Via};

    fn base_board() -> Board {
        let mut b = Board::new("t".into(), Resolution::new(Unit::Mil, 10000));
        b.layers = LayerStack::new(vec![
            Layer { name: "Top".into(), index: 0, is_signal: true, preferred: None },
            Layer { name: "Bot".into(), index: 1, is_signal: true, preferred: None },
        ]);
        b
    }

    #[test]
    fn picks_trace_near_segment() {
        let mut b = base_board();
        b.traces.push(Trace {
            layer: 0, width: 20,
            corners: vec![Point::new(0, 0), Point::new(1000, 0)],
            net: Some(3), fixed: FixedState::Route,
        });
        // 5 units above the centerline: within half-width(10)+tol
        let pick = pick_at(&b, Point::new(500, 5), 5.0, &[true, true]);
        assert_eq!(pick, Some(Pick::Trace { index: 0 }));
        // far away: no pick
        assert_eq!(pick_at(&b, Point::new(500, 500), 5.0, &[true, true]), None);
    }

    #[test]
    fn hidden_layer_trace_not_picked() {
        let mut b = base_board();
        b.traces.push(Trace {
            layer: 1, width: 20, corners: vec![Point::new(0, 0), Point::new(1000, 0)],
            net: Some(3), fixed: FixedState::Route,
        });
        assert_eq!(pick_at(&b, Point::new(500, 0), 5.0, &[true, false]), None);
    }

    #[test]
    fn pad_takes_priority_over_trace_when_overlapping() {
        let mut b = base_board();
        let pad = b.padstacks.add(Padstack {
            name: "P".into(),
            shapes: vec![Some(PadShape::Circle { radius: 30 }), None],
            drillable: false,
        });
        b.pins.push(Pin {
            component: "U1".into(), name: "1".into(), padstack: pad,
            location: Point::new(500, 0), net: Some(3), rotation: 0.0, front: true,
        });
        b.traces.push(Trace {
            layer: 0, width: 20, corners: vec![Point::new(0, 0), Point::new(1000, 0)],
            net: Some(3), fixed: FixedState::Route,
        });
        // query at the pad center: both contain it (dist 0); pad wins on priority
        assert_eq!(pick_at(&b, Point::new(500, 0), 5.0, &[true, true]), Some(Pick::Pad { pin_index: 0 }));
    }

    #[test]
    fn picks_via() {
        let mut b = base_board();
        let vp = b.padstacks.add(Padstack {
            name: "V".into(),
            shapes: vec![Some(PadShape::Circle { radius: 40 }), Some(PadShape::Circle { radius: 40 })],
            drillable: true,
        });
        b.vias.push(Via { padstack: vp, location: Point::new(200, 200), net: Some(1), fixed: FixedState::Route });
        assert_eq!(pick_at(&b, Point::new(210, 200), 5.0, &[true, true]), Some(Pick::Via { index: 0 }));
    }
}

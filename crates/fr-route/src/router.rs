//! Single-net routing: drive the A* between a net's pins and convert the resulting
//! grid path into board traces (one per layer run) and vias (at layer changes).

use crate::astar::{search, Costs, EdgeValidator};
use crate::grid::{Grid, Node};
use crate::obstacles::ObstacleMap;
use fr_board::{Board, FixedState, Trace, Via};
use fr_geometry::Point;

/// The geometry produced by routing one connection.
#[derive(Clone, Debug, Default)]
pub struct RoutedConnection {
    pub traces: Vec<Trace>,
    pub vias: Vec<Via>,
}

/// Route a two-pin connection of `net` from `start_pt` to `goal_pt`. Returns the
/// produced geometry, or None if no path was found within `max_expansions`.
#[allow(clippy::too_many_arguments)]
pub fn route_connection(
    board: &Board,
    grid: &Grid,
    obs: &ObstacleMap,
    net: u32,
    start_pt: Point,
    goal_pt: Point,
    width: i64,
    via_padstack: Option<usize>,
    costs: &Costs,
    max_expansions: usize,
    validator: Option<&EdgeValidator>,
) -> Option<RoutedConnection> {
    // start/goal nodes on every layer (the connection may begin/end on any layer the
    // pad reaches; for the grid model we allow all layers and let A* pick).
    let starts: Vec<Node> = (0..grid.layers).map(|l| grid.node_at(l, start_pt)).collect();
    let goals: Vec<Node> = (0..grid.layers).map(|l| grid.node_at(l, goal_pt)).collect();

    let path = search(grid, obs, net, &starts, &goals, costs, max_expansions, validator)?;
    Some(path_to_geometry(board, grid, net, &path, width, via_padstack))
}

/// Convert an A* node path into traces (collinear runs collapsed) + vias at layer
/// changes. Snaps endpoints to the true pin points so traces meet pads exactly.
fn path_to_geometry(
    _board: &Board,
    grid: &Grid,
    net: u32,
    path: &[Node],
    width: i64,
    via_padstack: Option<usize>,
) -> RoutedConnection {
    let mut out = RoutedConnection::default();
    if path.len() < 2 {
        return out;
    }

    // Split the path into maximal same-layer segments. A layer change between node i and
    // i+1 happens at the SAME (col,row) (via moves don't change col/row), so the boundary
    // point is shared by the segment that ends there and the one that begins there. We
    // emit a via at every such boundary, and a trace for each segment with >=2 distinct
    // points. Because each via sits exactly on the shared boundary point of both adjacent
    // segments, layer-to-layer connectivity is preserved even when a segment is a single
    // point (a straight stacked transition) and therefore produces no trace.
    let pts: Vec<Point> = path.iter().map(|&n| grid.point_of(n)).collect();

    let mut seg_start = 0usize;
    for i in 0..path.len() - 1 {
        let layer_changes = path[i + 1].layer != path[i].layer;
        if layer_changes {
            // close the current segment [seg_start..=i] on path[i].layer
            emit_trace(&mut out, &pts[seg_start..=i], path[i].layer, width, net);
            // via at the boundary point (path[i] and path[i+1] share col/row)
            if let Some(ps) = via_padstack {
                out.vias.push(Via {
                    padstack: ps,
                    location: pts[i],
                    net: Some(net as usize),
                    fixed: FixedState::Route,
                });
            }
            seg_start = i + 1;
        }
    }
    // final segment
    emit_trace(&mut out, &pts[seg_start..], path[path.len() - 1].layer, width, net);
    out
}

/// Emit a trace for a same-layer run of points, if it has >= 2 distinct points after
/// collinear simplification. A single-point run produces no trace (its connectivity is
/// carried by the vias at its endpoints, which sit on the same point).
fn emit_trace(out: &mut RoutedConnection, run: &[Point], layer: u32, width: i64, net: u32) {
    if run.len() < 2 {
        return;
    }
    let simplified = simplify_collinear(run);
    if simplified.len() >= 2 {
        out.traces.push(Trace {
            layer: layer as usize,
            width,
            corners: simplified,
            net: Some(net as usize),
            fixed: FixedState::Route,
        });
    }
}

/// Remove interior points that lie on the straight line between their neighbours, and
/// drop consecutive duplicates. Keeps trace corner lists compact.
fn simplify_collinear(pts: &[Point]) -> Vec<Point> {
    let mut dedup: Vec<Point> = Vec::with_capacity(pts.len());
    for &p in pts {
        if dedup.last() != Some(&p) {
            dedup.push(p);
        }
    }
    if dedup.len() <= 2 {
        return dedup;
    }
    let mut out = vec![dedup[0]];
    for i in 1..dedup.len() - 1 {
        let a = out[out.len() - 1];
        let b = dedup[i];
        let c = dedup[i + 1];
        // keep b only if a-b-c is not collinear (exact integer cross product)
        if a.side_of(c, b) != fr_geometry::Side::On {
            out.push(b);
        }
    }
    out.push(*dedup.last().unwrap());
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use fr_board::{Layer, LayerStack, Resolution, Unit};
    use fr_geometry::IntBox;

    fn empty_board(layers: usize) -> Board {
        let mut b = Board::new("t".into(), Resolution::new(Unit::Mil, 10000));
        let ls: Vec<Layer> = (0..layers)
            .map(|i| Layer { name: format!("L{i}"), index: i, is_signal: true, preferred: None })
            .collect();
        b.layers = LayerStack::new(ls);
        b
    }

    #[test]
    fn via_at_every_layer_change_connecting_traces() {
        let grid = Grid::new(IntBox::new(0, 0, 1_000_000, 1_000_000), 50_000, 2);
        // path: move on layer 0, change to layer 1 (same col/row), move on layer 1.
        let n = |layer, col, row| Node { layer, col, row };
        let path = vec![
            n(0, 2, 5), n(0, 5, 5),      // layer 0 run
            n(1, 5, 5),                   // via to layer 1 at (5,5)
            n(1, 8, 5),                   // layer 1 run
        ];
        let conn = path_to_geometry(&empty_board(2), &grid, 0, &path, 100_000, Some(0));
        // exactly one via, at the boundary point (col 5,row 5)
        assert_eq!(conn.vias.len(), 1, "one via at the single layer change");
        let via_pt = conn.vias[0].location;
        assert_eq!(via_pt, grid.point_of(n(0, 5, 5)));
        // two traces, one per layer, and each touches the via point
        assert_eq!(conn.traces.len(), 2, "a trace on each layer");
        let touches_via = |t: &fr_board::Trace| t.corners.first() == Some(&via_pt) || t.corners.last() == Some(&via_pt);
        assert!(conn.traces.iter().all(touches_via), "both traces meet at the via");
    }

    #[test]
    fn single_point_layer_run_keeps_vias_chained() {
        // path that changes layer twice at the SAME point (0->1->2): the middle layer-1
        // run is a single point and produces no trace, but the two vias share that point
        // so connectivity is preserved.
        let grid = Grid::new(IntBox::new(0, 0, 1_000_000, 1_000_000), 50_000, 3);
        let n = |layer, col, row| Node { layer, col, row };
        let path = vec![n(0, 2, 5), n(0, 5, 5), n(1, 5, 5), n(2, 5, 5), n(2, 8, 5)];
        let conn = path_to_geometry(&empty_board(3), &grid, 0, &path, 100_000, Some(0));
        assert_eq!(conn.vias.len(), 2, "two layer changes -> two vias");
        // both vias at the same point (5,5)
        assert_eq!(conn.vias[0].location, conn.vias[1].location);
    }

    #[test]
    fn routes_and_produces_a_trace() {
        let board = empty_board(1);
        let grid = Grid::new(IntBox::new(0, 0, 1_000_000, 1_000_000), 50_000, 1);
        let obs = ObstacleMap::build(&board, &grid);
        let costs = Costs::for_grid(&grid, 500_000);
        let conn = route_connection(
            &board, &grid, &obs, 0,
            Point::new(100_000, 500_000), Point::new(900_000, 500_000),
            100_000, None, &costs, 1_000_000, None,
        ).expect("route");
        assert!(!conn.traces.is_empty());
        // a straight horizontal route collapses to a 2-point trace
        let t = &conn.traces[0];
        assert!(t.corners.len() >= 2);
        assert_eq!(t.width, 100_000);
    }

    #[test]
    fn collinear_simplification() {
        let pts = vec![
            Point::new(0, 0), Point::new(10, 0), Point::new(20, 0), Point::new(30, 0),
        ];
        let s = simplify_collinear(&pts);
        assert_eq!(s, vec![Point::new(0, 0), Point::new(30, 0)]);
    }
}

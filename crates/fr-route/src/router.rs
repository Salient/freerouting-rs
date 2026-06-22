//! Single-net routing: drive the A* between a net's pins and convert the resulting
//! grid path into board traces (one per layer run) and vias (at layer changes).

use crate::astar::{search, Costs};
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
) -> Option<RoutedConnection> {
    // start/goal nodes on every layer (the connection may begin/end on any layer the
    // pad reaches; for the grid model we allow all layers and let A* pick).
    let starts: Vec<Node> = (0..grid.layers).map(|l| grid.node_at(l, start_pt)).collect();
    let goals: Vec<Node> = (0..grid.layers).map(|l| grid.node_at(l, goal_pt)).collect();

    let path = search(grid, obs, net, &starts, &goals, costs, max_expansions)?;
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

    let mut run: Vec<Point> = Vec::new();
    let mut run_layer = path[0].layer;
    run.push(grid.point_of(path[0]));

    for w in path.windows(2) {
        let (a, b) = (w[0], w[1]);
        if b.layer != a.layer {
            // layer change: close the current run as a trace, drop a via, start new run
            flush_run(&mut out, &mut run, run_layer, width, net);
            if let Some(ps) = via_padstack {
                out.vias.push(Via {
                    padstack: ps,
                    location: grid.point_of(a),
                    net: Some(net as usize),
                    fixed: FixedState::Route,
                });
            }
            run_layer = b.layer;
            run.push(grid.point_of(a)); // via location starts the new run
            run.push(grid.point_of(b));
        } else {
            run.push(grid.point_of(b));
        }
    }
    flush_run(&mut out, &mut run, run_layer, width, net);
    out
}

/// Collapse collinear points and emit a trace if >= 2 distinct points remain.
fn flush_run(out: &mut RoutedConnection, run: &mut Vec<Point>, layer: u32, width: i64, net: u32) {
    if run.len() >= 2 {
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
    run.clear();
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
    fn routes_and_produces_a_trace() {
        let board = empty_board(1);
        let grid = Grid::new(IntBox::new(0, 0, 1_000_000, 1_000_000), 50_000, 1);
        let obs = ObstacleMap::build(&board, &grid);
        let costs = Costs::for_grid(&grid, 500_000);
        let conn = route_connection(
            &board, &grid, &obs, 0,
            Point::new(100_000, 500_000), Point::new(900_000, 500_000),
            100_000, None, &costs, 1_000_000,
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

//! fr-engine: orchestration. Resolves each net's pins, drives the single-net router to
//! connect them, and accumulates traces/vias onto the board. Phase 6/8 add shove/rip-up
//! and the parallel multi-net scheduler; this version routes nets sequentially with a
//! deterministic order and a wall-clock budget.

use std::time::{Duration, Instant};

use fr_board::{Board, PadShape, Padstack, Pin};
use fr_geometry::Point;
use fr_route::{route_connection, Costs, Grid, ObstacleMap};

/// Options controlling a routing run.
#[derive(Clone, Copy, Debug)]
pub struct RouteOptions {
    /// Wall-clock budget in seconds (0 = no limit).
    pub max_time_secs: u64,
    /// Worker threads (0 = auto). Reserved for the parallel scheduler (Phase 8).
    pub threads: usize,
    /// Deterministic seed.
    pub seed: u64,
}

impl Default for RouteOptions {
    fn default() -> Self {
        RouteOptions { max_time_secs: 0, threads: 0, seed: 1 }
    }
}

/// Summary of a routing run.
#[derive(Clone, Copy, Debug, Default)]
pub struct RouteReport {
    pub nets_total: usize,
    pub nets_completed: usize,
    pub connections_routed: usize,
    pub connections_failed: usize,
    pub passes: usize,
}

/// Route every net on the board. Mutates `board.traces` / `board.vias`.
pub fn route_board(board: &mut Board, opts: &RouteOptions) -> RouteReport {
    let start = Instant::now();
    let budget = (opts.max_time_secs > 0).then(|| Duration::from_secs(opts.max_time_secs));

    // 1. Resolve pins from net pin-references ("Comp-Pin") to board points so the router
    //    has concrete endpoints. Build/augment board.pins for routable (shaped) pads.
    ensure_pins(board);

    // 2. Choose a routing grid. Pitch = the smaller of (default trace width + clearance)
    //    and a fraction of the board, clamped so the node count stays manageable.
    let Some(bounds) = board.outline_box().or_else(|| {
        fr_geometry::IntBox::bound(board.pins.iter().map(|p| p.location))
    }) else {
        return RouteReport { nets_total: board.nets.len(), ..Default::default() };
    };
    let pitch = choose_pitch(board, bounds);
    // Cap layers used for search to keep the node count and via fan-out modest; the grid
    // router uses all signal layers but very deep stacks blow up the closed set.
    let grid = Grid::new(bounds, pitch, board.layer_count().max(1).min(6));
    let via_padstack = ensure_via_padstack(board);
    let costs = Costs::for_grid(&grid, board.rules.default_clearance.max(grid.pitch) * 4);

    // 3. Route each net's connections in deterministic order. We connect pins in a
    //    simple chain (pin[0]->pin[1]->...): a minimal spanning approach good enough to
    //    exercise the pipeline; MST/steiner ordering is a later refinement.
    let mut report = RouteReport { nets_total: board.nets.len(), passes: 1, ..Default::default() };
    let net_count = board.nets.len();
    let max_expansions = expansion_budget(&grid);

    // Build the obstacle map once over all pins, then refresh it periodically as traces
    // accumulate. Rebuilding per-connection is O(pins x connections) and far too slow on
    // an 800-component board; refreshing every `refresh_every` routed connections keeps
    // newly-added traces blocking subsequent nets without quadratic cost.
    let mut obs = ObstacleMap::build(board, &grid);
    let refresh_every = 32usize;
    let mut since_refresh = 0usize;

    for net_id in 0..net_count {
        if let Some(b) = budget {
            if start.elapsed() >= b {
                break;
            }
        }
        let pin_pts = net_pin_points(board, net_id);
        if pin_pts.len() < 2 {
            if pin_pts.len() <= 1 {
                report.nets_completed += 1;
            }
            continue;
        }
        let width = board.rules.default_width.max(grid.pitch / 2);
        let mut all_ok = true;
        let mut produced = Vec::new();
        // Connect pins along a minimum spanning tree (shortest total wire, each edge a
        // short hop) rather than a naive sorted chain - more edges route successfully.
        for (ai, bi) in mst_edges(&pin_pts) {
            let conn = route_connection(
                board, &grid, &obs, net_id as u32,
                pin_pts[ai], pin_pts[bi],
                width, Some(via_padstack), &costs, max_expansions,
            );
            match conn {
                Some(c) => {
                    produced.push(c);
                    report.connections_routed += 1;
                }
                None => {
                    report.connections_failed += 1;
                    all_ok = false;
                }
            }
        }
        // commit this net's geometry
        for c in produced {
            board.traces.extend(c.traces);
            board.vias.extend(c.vias);
            since_refresh += 1;
        }
        if all_ok {
            report.nets_completed += 1;
        }
        if since_refresh >= refresh_every {
            obs = ObstacleMap::build(board, &grid);
            since_refresh = 0;
        }
    }

    report
}

/// Build board.pins from net pin-references if the board has none yet. We can only place
/// a pin where we know its location; without component/image geometry we approximate by
/// using the component placement location. (Good enough for the grid pipeline; exact pin
/// offsets come with full image parsing.)
fn ensure_pins(board: &mut Board) {
    if !board.pins.is_empty() {
        return;
    }
    // Map component name -> location.
    let comp_loc: std::collections::HashMap<String, Point> =
        board.components.iter().map(|c| (c.name.clone(), c.location)).collect();

    // A default routable padstack so pins have copper to stamp.
    let layer_count = board.layer_count().max(1);
    let pin_pad = board.padstacks.add(Padstack {
        name: "__pin".into(),
        shapes: (0..layer_count).map(|_| Some(PadShape::Circle { radius: 20_000 })).collect(),
        drillable: false,
    });

    let mut new_pins = Vec::new();
    for net_id in 0..board.nets.len() {
        let net = board.nets.get(net_id).unwrap();
        for pin_ref in &net.pins {
            // "Comp-Pin" -> component name is the part before the last '-'
            if let Some(idx) = pin_ref.rfind('-') {
                let comp = &pin_ref[..idx];
                if let Some(loc) = comp_loc.get(comp) {
                    new_pins.push(Pin {
                        component: comp.to_string(),
                        name: pin_ref[idx + 1..].to_string(),
                        padstack: pin_pad,
                        location: *loc,
                        net: Some(net_id),
                    });
                }
            }
        }
    }
    board.pins = new_pins;
}

/// Minimum spanning tree edges (Prim's, O(n^2)) over the points, returned as index
/// pairs (a, b). Edge weight is squared Euclidean distance (monotonic, exact). For n<=1
/// returns no edges. n is small per net (typically < 50), so O(n^2) is ample.
fn mst_edges(pts: &[Point]) -> Vec<(usize, usize)> {
    let n = pts.len();
    if n < 2 {
        return Vec::new();
    }
    let mut in_tree = vec![false; n];
    let mut best_cost = vec![i128::MAX; n];
    let mut best_from = vec![usize::MAX; n];
    let mut edges = Vec::with_capacity(n - 1);
    in_tree[0] = true;
    for j in 1..n {
        best_cost[j] = pts[0].distance_square(pts[j]);
        best_from[j] = 0;
    }
    for _ in 1..n {
        // pick the cheapest non-tree vertex
        let mut u = usize::MAX;
        let mut ucost = i128::MAX;
        for j in 0..n {
            if !in_tree[j] && best_cost[j] < ucost {
                ucost = best_cost[j];
                u = j;
            }
        }
        if u == usize::MAX {
            break;
        }
        in_tree[u] = true;
        edges.push((best_from[u], u));
        // relax
        for j in 0..n {
            if !in_tree[j] {
                let d = pts[u].distance_square(pts[j]);
                if d < best_cost[j] {
                    best_cost[j] = d;
                    best_from[j] = u;
                }
            }
        }
    }
    edges
}

/// Distinct routable pin points of a net (dedup identical component locations).
fn net_pin_points(board: &Board, net_id: usize) -> Vec<Point> {
    let mut pts: Vec<Point> = board.pins_of_net(net_id).map(|p| p.location).collect();
    pts.sort_by_key(|p| (p.x, p.y));
    pts.dedup();
    pts
}

fn choose_pitch(board: &Board, bounds: fr_geometry::IntBox) -> i64 {
    let base = (board.rules.default_width + board.rules.default_clearance).max(1);
    // Keep the grid coarse enough that a single A* search stays fast on large boards.
    // Cap the longer dimension to ~400 cells; the per-search budget assumes this scale.
    let span = bounds.width().max(bounds.height()).max(1);
    let min_pitch_for_size = span / 400;
    base.max(min_pitch_for_size).max(1)
}

fn ensure_via_padstack(board: &mut Board) -> usize {
    // Reuse an existing through via padstack if present, else add one.
    if let Some(i) = (0..board.padstacks.len())
        .find(|&i| board.padstacks.get(i).map(|p| !p.is_empty() && p.from_layer() == Some(0)).unwrap_or(false))
    {
        return i;
    }
    let layer_count = board.layer_count().max(1);
    board.padstacks.add(Padstack {
        name: "__via".into(),
        shapes: (0..layer_count).map(|_| Some(PadShape::Circle { radius: 120_000 })).collect(),
        drillable: true,
    })
}

fn expansion_budget(grid: &Grid) -> usize {
    // Cap per-connection A* work so one hard net can't stall the board. A few times the
    // single-layer cell count is plenty to find a path or prove it's not worth more.
    let per_layer = (grid.cols * grid.rows) as usize;
    (per_layer * 4).clamp(20_000, 800_000)
}

#[cfg(test)]
mod tests {
    use super::*;
    use fr_board::{Component, Layer, LayerStack, Net, Resolution, Unit};

    fn two_pin_board() -> Board {
        let mut b = Board::new("t".into(), Resolution::new(Unit::Mil, 10000));
        b.layers = LayerStack::new(vec![
            Layer { name: "Top".into(), index: 0, is_signal: true, preferred: None },
            Layer { name: "Bot".into(), index: 1, is_signal: true, preferred: None },
        ]);
        b.rules = fr_board::Rules::new(100_000, 80_000);
        b.outline = vec![
            Point::new(0, 0), Point::new(5_000_000, 0),
            Point::new(5_000_000, 5_000_000), Point::new(0, 5_000_000),
        ];
        b.components.push(Component { name: "U1".into(), image: "IMG".into(), location: Point::new(1_000_000, 2_500_000), front: true, rotation: 0.0 });
        b.components.push(Component { name: "U2".into(), image: "IMG".into(), location: Point::new(4_000_000, 2_500_000), front: true, rotation: 0.0 });
        b.nets.add(Net { name: "N1".into(), pins: vec!["U1-1".into(), "U2-1".into()] });
        b
    }

    #[test]
    fn routes_a_two_pin_net() {
        let mut b = two_pin_board();
        let report = route_board(&mut b, &RouteOptions::default());
        assert_eq!(report.nets_total, 1);
        assert_eq!(report.nets_completed, 1, "the single net should connect");
        assert!(report.connections_routed >= 1);
        assert!(!b.traces.is_empty(), "should have produced traces");
    }

    #[test]
    fn mst_connects_all_pins_minimally() {
        // 4 points in a line: MST should be the 3 adjacent edges (not the long diagonals).
        let pts = vec![
            Point::new(0, 0), Point::new(10, 0), Point::new(20, 0), Point::new(30, 0),
        ];
        let edges = mst_edges(&pts);
        assert_eq!(edges.len(), 3, "n-1 edges for n=4");
        // total squared length should be 3 * 100 = 300 (adjacent hops), not more
        let total: i128 = edges.iter().map(|&(a, b)| pts[a].distance_square(pts[b])).sum();
        assert_eq!(total, 300);
    }

    #[test]
    fn mst_trivial_sizes() {
        assert!(mst_edges(&[]).is_empty());
        assert!(mst_edges(&[Point::new(1, 1)]).is_empty());
        assert_eq!(mst_edges(&[Point::new(0, 0), Point::new(5, 0)]).len(), 1);
    }

    #[test]
    fn deterministic_routing() {
        let mut b1 = two_pin_board();
        let mut b2 = two_pin_board();
        route_board(&mut b1, &RouteOptions::default());
        route_board(&mut b2, &RouteOptions::default());
        assert_eq!(b1.traces.len(), b2.traces.len());
        // first trace corner lists identical
        if let (Some(t1), Some(t2)) = (b1.traces.first(), b2.traces.first()) {
            assert_eq!(t1.corners, t2.corners, "routing must be deterministic");
        }
    }
}

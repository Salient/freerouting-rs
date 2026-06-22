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

    // 3. Route nets. Connections within a net follow an MST over its pins. Routing runs
    //    in parallel by default (one shared obstacle snapshot, deterministic ordered
    //    commit with conflict repair); threads==1 forces the sequential path.
    let mut report = RouteReport { nets_total: board.nets.len(), passes: 1, ..Default::default() };
    let max_expansions = expansion_budget(&grid);

    // Build the obstacle map once over all pins, then refresh it periodically as traces
    // accumulate. Rebuilding per-connection is O(pins x connections) and far too slow on
    // an 800-component board; refreshing every `refresh_every` routed connections keeps
    // newly-added traces blocking subsequent nets without quadratic cost.
    let ctx = RouteCtx { grid: &grid, via_padstack, costs, max_expansions,
                         width: board.rules.default_width.max(grid.pitch / 2) };

    if opts.threads != 1 {
        route_parallel(board, &ctx, opts, start, budget, &mut report);
    } else {
        route_sequential(board, &ctx, start, budget, &mut report);
    }
    report
}

/// Per-run routing context shared across nets (immutable).
struct RouteCtx<'a> {
    grid: &'a Grid,
    via_padstack: usize,
    costs: Costs,
    max_expansions: usize,
    width: i64,
}

/// Geometry produced for one net (not yet committed to the board).
struct NetResult {
    traces: Vec<fr_board::Trace>,
    vias: Vec<fr_board::Via>,
    connections_routed: usize,
    connections_failed: usize,
    fully_routed: bool,
}

/// Route a single net's MST edges against an obstacle snapshot. Pure: no board mutation.
fn route_one_net(
    board: &Board,
    obs: &ObstacleMap,
    ctx: &RouteCtx,
    net_id: usize,
) -> Option<NetResult> {
    let pin_pts = net_pin_points(board, net_id);
    if pin_pts.len() < 2 {
        return None; // nothing to connect (caller counts trivially-complete nets)
    }
    let mut res = NetResult {
        traces: Vec::new(), vias: Vec::new(),
        connections_routed: 0, connections_failed: 0, fully_routed: true,
    };
    for (ai, bi) in mst_edges(&pin_pts) {
        match route_connection(
            board, ctx.grid, obs, net_id as u32,
            pin_pts[ai], pin_pts[bi],
            ctx.width, Some(ctx.via_padstack), &ctx.costs, ctx.max_expansions,
        ) {
            Some(c) => {
                res.traces.extend(c.traces);
                res.vias.extend(c.vias);
                res.connections_routed += 1;
            }
            None => {
                res.connections_failed += 1;
                res.fully_routed = false;
            }
        }
    }
    Some(res)
}

/// Sequential router: route nets in id order, refreshing the obstacle map periodically.
fn route_sequential(
    board: &mut Board,
    ctx: &RouteCtx,
    start: Instant,
    budget: Option<Duration>,
    report: &mut RouteReport,
) {
    let net_count = board.nets.len();
    let mut obs = ObstacleMap::build(board, ctx.grid);
    let refresh_every = 32usize;
    let mut since_refresh = 0usize;

    for net_id in 0..net_count {
        if budget.map(|b| start.elapsed() >= b).unwrap_or(false) {
            break;
        }
        match route_one_net(board, &obs, ctx, net_id) {
            None => {
                report.nets_completed += 1; // 0/1-pin net is trivially complete
            }
            Some(r) => {
                report.connections_routed += r.connections_routed;
                report.connections_failed += r.connections_failed;
                if r.fully_routed {
                    report.nets_completed += 1;
                }
                let added = r.traces.len() + r.vias.len();
                board.traces.extend(r.traces);
                board.vias.extend(r.vias);
                since_refresh += added;
                if since_refresh >= refresh_every {
                    obs = ObstacleMap::build(board, ctx.grid);
                    since_refresh = 0;
                }
            }
        }
    }
}

/// Parallel router (Phase 8). Routes nets concurrently against a shared immutable
/// obstacle snapshot via rayon, then commits results in deterministic net-id order,
/// detecting conflicts where a later net's geometry overlaps an already-committed net.
/// Conflicting nets are re-routed sequentially in a second pass against the updated
/// board. Determinism: results are gathered into net-id order before committing, and
/// the conflict pass is sequential, so output is independent of thread scheduling.
fn route_parallel(
    board: &mut Board,
    ctx: &RouteCtx,
    opts: &RouteOptions,
    start: Instant,
    budget: Option<Duration>,
    report: &mut RouteReport,
) {
    use rayon::prelude::*;

    let net_count = board.nets.len();
    let pool = build_pool(opts.threads);

    // Phase A: route every net in parallel against one shared snapshot.
    let snapshot = ObstacleMap::build(board, ctx.grid);
    let results: Vec<(usize, Option<NetResult>)> = pool.install(|| {
        (0..net_count)
            .into_par_iter()
            .map(|net_id| (net_id, route_one_net(board, &snapshot, ctx, net_id)))
            .collect()
    });

    // Phase B: deterministic commit in net-id order with conflict detection. Because all
    // nets saw the same snapshot, two nets may claim overlapping cells; the first (lower
    // id) wins, later conflicters are deferred to the sequential repair pass.
    let mut occupied: std::collections::HashSet<(u32, i32, i32)> = std::collections::HashSet::new();
    let mut deferred: Vec<usize> = Vec::new();

    for (net_id, maybe) in results {
        match maybe {
            None => report.nets_completed += 1,
            Some(r) => {
                if conflicts(&r, ctx, &occupied) {
                    deferred.push(net_id);
                } else {
                    mark_occupied(&r, ctx, &mut occupied);
                    report.connections_routed += r.connections_routed;
                    report.connections_failed += r.connections_failed;
                    if r.fully_routed {
                        report.nets_completed += 1;
                    }
                    board.traces.extend(r.traces);
                    board.vias.extend(r.vias);
                }
            }
        }
    }

    // Phase C: sequential repair of conflicting nets against the now-populated board.
    let mut obs = ObstacleMap::build(board, ctx.grid);
    let mut since_refresh = 0usize;
    for net_id in deferred {
        if budget.map(|b| start.elapsed() >= b).unwrap_or(false) {
            break;
        }
        if let Some(r) = route_one_net(board, &obs, ctx, net_id) {
            report.connections_routed += r.connections_routed;
            report.connections_failed += r.connections_failed;
            if r.fully_routed {
                report.nets_completed += 1;
            }
            since_refresh += r.traces.len() + r.vias.len();
            board.traces.extend(r.traces);
            board.vias.extend(r.vias);
            if since_refresh >= 32 {
                obs = ObstacleMap::build(board, ctx.grid);
                since_refresh = 0;
            }
        }
    }
}

fn build_pool(threads: usize) -> rayon::ThreadPool {
    let mut b = rayon::ThreadPoolBuilder::new();
    if threads > 0 {
        b = b.num_threads(threads);
    }
    b.build().unwrap_or_else(|_| rayon::ThreadPoolBuilder::new().build().unwrap())
}

/// Grid cells a net result occupies (trace corners + via locations), as (layer,col,row).
fn result_cells(r: &NetResult, ctx: &RouteCtx) -> Vec<(u32, i32, i32)> {
    let mut cells = Vec::new();
    for t in &r.traces {
        for p in &t.corners {
            let n = ctx.grid.node_at(t.layer, *p);
            cells.push((n.layer, n.col, n.row));
        }
    }
    for v in &r.vias {
        let n = ctx.grid.node_at(0, v.location);
        cells.push((n.layer, n.col, n.row));
    }
    cells
}

fn conflicts(r: &NetResult, ctx: &RouteCtx, occupied: &std::collections::HashSet<(u32, i32, i32)>) -> bool {
    result_cells(r, ctx).iter().any(|c| occupied.contains(c))
}

fn mark_occupied(r: &NetResult, ctx: &RouteCtx, occupied: &mut std::collections::HashSet<(u32, i32, i32)>) {
    for c in result_cells(r, ctx) {
        occupied.insert(c);
    }
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
    fn parallel_routes_the_two_pin_net() {
        let mut b = two_pin_board();
        let report = route_board(&mut b, &RouteOptions { max_time_secs: 0, threads: 0, seed: 1 });
        assert_eq!(report.nets_completed, 1, "parallel path should connect the net");
        assert!(!b.traces.is_empty());
    }

    #[test]
    fn parallel_is_deterministic() {
        let mut b1 = two_pin_board();
        let mut b2 = two_pin_board();
        route_board(&mut b1, &RouteOptions { max_time_secs: 0, threads: 4, seed: 1 });
        route_board(&mut b2, &RouteOptions { max_time_secs: 0, threads: 4, seed: 1 });
        assert_eq!(b1.traces.len(), b2.traces.len());
        if let (Some(t1), Some(t2)) = (b1.traces.first(), b2.traces.first()) {
            assert_eq!(t1.corners, t2.corners, "parallel routing must be deterministic");
        }
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

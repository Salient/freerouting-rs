//! fr-engine: orchestration. Resolves each net's pins, drives the single-net router to
//! connect them, and accumulates traces/vias onto the board. Phase 6/8 add shove/rip-up
//! and the parallel multi-net scheduler; this version routes nets sequentially with a
//! deterministic order and a wall-clock budget.

use std::time::{Duration, Instant};

pub mod interactive;

pub use fr_route::AngleRestriction;
pub use interactive::InteractiveRouter;

use fr_board::{Board, PadShape, Padstack, Pin};
use fr_geometry::{point_seg_dist, seg_seg_dist, Point};
use fr_route::{route_connection, Costs, EdgeValidator, Grid, ObstacleIndex, ObstacleMap, NO_NET};

/// Options controlling a routing run.
#[derive(Clone, Copy, Debug)]
pub struct RouteOptions {
    /// Wall-clock budget in seconds (0 = no limit).
    pub max_time_secs: u64,
    /// Worker threads (0 = auto). Reserved for the parallel scheduler (Phase 8).
    pub threads: usize,
    /// Deterministic seed.
    pub seed: u64,
    /// Override trace width in board units (0 = use the board's rule width).
    pub width: i64,
    /// Override clearance in board units (0 = use the board's rule clearance).
    pub clearance: i64,
    /// Max signal layers to route on (0 = all).
    pub max_layers: usize,
}

impl Default for RouteOptions {
    fn default() -> Self {
        RouteOptions { max_time_secs: 0, threads: 0, seed: 1, width: 0, clearance: 0, max_layers: 0 }
    }
}

/// Summary of a routing run.
#[derive(Clone, Debug, Default)]
pub struct RouteReport {
    pub nets_total: usize,
    pub nets_completed: usize,
    pub connections_routed: usize,
    pub connections_failed: usize,
    pub passes: usize,
    /// Net ids that did not fully route (for ratsnest / incompletes display).
    pub unrouted_nets: Vec<usize>,
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
    // Apply option overrides to the board rules (width/clearance) before sizing the grid.
    if opts.width > 0 {
        board.rules.default_width = opts.width;
    }
    if opts.clearance > 0 {
        board.rules.default_clearance = opts.clearance;
        board.rules.edge_clearance = opts.clearance;
    }
    let pitch = choose_pitch(board, bounds);
    // Cap layers used for search to keep the node count and via fan-out modest; the grid
    // router uses all signal layers but very deep stacks blow up the closed set.
    let layer_cap = if opts.max_layers > 0 { opts.max_layers } else { 6 };
    let grid = Grid::new(bounds, pitch, board.layer_count().max(1).min(layer_cap));
    let via_padstack = ensure_via_padstack(board);
    let costs = Costs::for_grid(&grid, board.rules.default_clearance.max(grid.pitch) * 4);

    // 3. Route nets incrementally: connections within a net follow an MST over its pins,
    //    and each routed connection is stamped into a shared mutable obstacle map right
    //    away, so every later connection (and net) sees it and routes around it with the
    //    design clearance. This is what guarantees no two different-net traces overlap
    //    (the previous parallel "clean snapshot" approach shorted nets together).
    let mut report = RouteReport { nets_total: board.nets.len(), passes: 1, ..Default::default() };
    let max_expansions = expansion_budget(&grid);
    let ctx = RouteCtx { grid: &grid, via_padstack, costs, max_expansions,
                         width: board.rules.default_width.max(grid.pitch / 2),
                         clearance: board.rules.default_clearance };

    route_incremental(board, &ctx, start, budget, &mut report);
    report
}

/// One trace segment, flattened for DRC: layer, net, endpoints, half-width.
struct Seg {
    layer: usize,
    net: usize,
    a: Point,
    b: Point,
    half: f64,
}

fn flatten_segments(board: &Board) -> Vec<Seg> {
    let mut segs = Vec::new();
    for t in &board.traces {
        let net = t.net.unwrap_or(usize::MAX);
        let half = (t.width as f64) / 2.0;
        for w in t.corners.windows(2) {
            segs.push(Seg { layer: t.layer, net, a: w[0], b: w[1], half });
        }
    }
    segs
}

/// True copper-geometry DRC: count pairs of different-net trace segments on the same
/// layer whose copper actually overlaps (segment-to-segment distance < sum of half
/// widths). This is the honest short check - the previous centerline-cell version missed
/// overlaps caused by trace WIDTH. O(n^2) over segments via a coarse spatial bucket.
pub fn drc_short_count(board: &Board) -> usize {
    drc_violation_count(board, 0.0)
}

/// Count different-net same-layer segment pairs closer than (half_a + half_b + extra).
/// extra=0 => actual copper overlap (shorts); extra=clearance => clearance violations.
pub fn drc_violation_count(board: &Board, extra: f64) -> usize {
    use std::collections::HashMap;
    let segs = flatten_segments(board);
    // bucket segments by layer + coarse cell of their midpoint to limit pair tests
    let bucket = (board.rules.default_width + board.rules.default_clearance).max(1) * 4;
    let mut grid: HashMap<(usize, i64, i64), Vec<usize>> = HashMap::new();
    let cell_of = |layer: usize, p: Point| (layer, p.x.div_euclid(bucket), p.y.div_euclid(bucket));
    for (i, s) in segs.iter().enumerate() {
        // register in all buckets the segment's bbox (expanded) touches
        let minx = s.a.x.min(s.b.x).div_euclid(bucket);
        let maxx = s.a.x.max(s.b.x).div_euclid(bucket);
        let miny = s.a.y.min(s.b.y).div_euclid(bucket);
        let maxy = s.a.y.max(s.b.y).div_euclid(bucket);
        for cx in minx..=maxx {
            for cy in miny..=maxy {
                grid.entry((s.layer, cx, cy)).or_default().push(i);
            }
        }
        let _ = cell_of;
    }
    let mut violations = 0usize;
    let mut seen: std::collections::HashSet<(usize, usize)> = std::collections::HashSet::new();
    for ids in grid.values() {
        for (xi, &i) in ids.iter().enumerate() {
            for &j in &ids[xi + 1..] {
                if i == j {
                    continue;
                }
                let (si, sj) = (&segs[i], &segs[j]);
                if si.net == sj.net || si.layer != sj.layer {
                    continue;
                }
                let pair = (i.min(j), i.max(j));
                if !seen.insert(pair) {
                    continue;
                }
                let d = seg_seg_dist(si.a, si.b, sj.a, sj.b);
                if d < si.half + sj.half + extra - 1.0 {
                    violations += 1;
                }
            }
        }
    }
    violations
}

/// Count trace segments that pass within copper distance of a DIFFERENT-net pin on a
/// layer the pin's pad covers. This catches traces shorting to component pads (the
/// trace-vs-pad case the segment-vs-segment check misses).
pub fn drc_trace_pin_short_count(board: &Board) -> usize {
    let segs = flatten_segments(board);
    let mut shorts = 0usize;
    for pin in &board.pins {
        let Some(ps) = board.padstacks.get(pin.padstack) else { continue };
        if ps.is_empty() {
            continue;
        }
        let pin_net = pin.net.unwrap_or(usize::MAX);
        // Circumscribed radius across all pad shapes (circle radius or convex pad's
        // farthest vertex), matching the obstacle index the router avoids. So the router
        // keeps traces clear of this disc and the DRC checks the same disc -> consistent.
        let pad_r = ps
            .shapes
            .iter()
            .filter_map(|s| pad_shape_extent(s.as_ref()))
            .fold(0i64, i64::max) as f64;
        let (lo, hi) = (ps.from_layer().unwrap_or(0), ps.to_layer().unwrap_or(0));
        for s in &segs {
            if s.net == pin_net || s.layer < lo || s.layer > hi {
                continue;
            }
            let d = point_seg_dist(pin.location, s.a, s.b);
            if d < pad_r + s.half - 1.0 {
                shorts += 1;
            }
        }
    }
    shorts
}


/// Per-run routing context shared across nets (immutable).
struct RouteCtx<'a> {
    grid: &'a Grid,
    via_padstack: usize,
    costs: Costs,
    max_expansions: usize,
    width: i64,
    clearance: i64,
}

/// Build the exact-geometry obstacle index from the board's static copper (pads/vias and
/// any pre-existing traces). This mirrors what the coarse `ObstacleMap` stamps, but keeps
/// EXACT geometry so the A* edge validator can reject a trace segment that clips a pad
/// between two passable grid cells (the trace-to-pad short the grid alone can't avoid).
/// The circumscribed radius of a pad shape relative to its origin: the circle radius, or
/// a convex pad's farthest vertex distance from the origin. None for an empty slot.
fn pad_shape_extent(shape: Option<&PadShape>) -> Option<i64> {
    match shape? {
        PadShape::Circle { radius } => Some(*radius),
        PadShape::Convex(tile) => {
            let r2 = tile
                .vertices()
                .iter()
                .map(|v| (v.x as i128) * (v.x as i128) + (v.y as i128) * (v.y as i128))
                .max()
                .unwrap_or(0);
            Some((r2 as f64).sqrt().ceil() as i64)
        }
    }
}

pub fn build_obstacle_index(board: &Board, layers: usize) -> ObstacleIndex {
    let mut idx = ObstacleIndex::new(layers);
    for pin in &board.pins {
        let Some(ps) = board.padstacks.get(pin.padstack) else { continue };
        if ps.is_empty() {
            continue;
        }
        let net = pin.net.map(|n| n as u32).unwrap_or(NO_NET);
        let (lo, hi) = (ps.from_layer().unwrap_or(0), ps.to_layer().unwrap_or(layers - 1));
        // A circumscribing radius covering ANY pad shape on the padstack (circle radius, or
        // a convex pad's farthest vertex from the pad origin). Used as the fallback so a
        // pad that has copper on a layer always stamps an obstacle that fully covers it —
        // critically, rectangular/polygon-only pads (no circle shape) must NOT stamp a
        // zero-radius disc (which would let traces cut through them).
        let max_r = ps
            .shapes
            .iter()
            .filter_map(|s| pad_shape_extent(s.as_ref()))
            .fold(0i64, i64::max);
        for layer in lo..=hi.min(layers - 1) {
            let r = match ps.shapes.get(layer).and_then(|s| s.as_ref()) {
                Some(PadShape::Circle { radius }) => *radius,
                // Convex pad: a disc of its circumscribed radius conservatively covers the
                // true copper (over-reserves slightly at corners; never under-reserves).
                Some(PadShape::Convex(_)) => max_r,
                None => max_r, // no shape this layer but the padstack has copper elsewhere
            };
            if r > 0 {
                idx.add_disc(layer, pin.location, r, net);
            }
        }
    }
    for t in &board.traces {
        let net = t.net.map(|n| n as u32).unwrap_or(NO_NET);
        idx.add_trace(t.layer, &t.corners, t.width, net);
    }
    for v in &board.vias {
        let net = v.net.map(|n| n as u32).unwrap_or(NO_NET);
        let r = fr_route::via_radius(board, v.padstack, 0);
        if r > 0 {
            idx.add_via(0, layers - 1, v.location, r, net);
        }
    }
    idx.build();
    idx
}

/// Incremental router: route nets in id order against a single mutable obstacle map,
/// stamping each routed connection (with width + clearance) immediately so subsequent
/// connections and nets must route around it. This guarantees different-net traces keep
/// clearance and never overlap (no shorts). Deterministic: net order and the A* search
/// are deterministic, and there is no thread nondeterminism.
fn route_incremental(
    board: &mut Board,
    ctx: &RouteCtx,
    start: Instant,
    budget: Option<Duration>,
    report: &mut RouteReport,
) {
    let net_count = board.nets.len();

    // Route order: smallest-bounding-box (most local/constrained) nets first, so tight
    // connections claim their short paths before long nets sprawl across the board. Nets
    // with <2 pins are trivially complete.
    let mut order: Vec<usize> = (0..net_count).collect();
    let bbox_span = |nid: usize| -> i64 {
        let pts = net_pin_points(board, nid);
        fr_geometry::IntBox::bound(pts.iter().copied())
            .map(|b| b.width() + b.height())
            .unwrap_or(0)
    };
    let spans: Vec<i64> = (0..net_count).map(bbox_span).collect();
    order.sort_by_key(|&nid| spans[nid]);

    let mut obs = ObstacleMap::build_with_clearance(board, ctx.grid, ctx.clearance);
    // Exact-geometry obstacle index, used by the A* edge validator to reject trace
    // segments that would clip a different-net pad/trace between two passable grid cells.
    // Kept in lock-step with `obs`: every committed trace/via is added to BOTH.
    let mut index = build_obstacle_index(board, ctx.grid.layers);
    let mut completed = vec![false; net_count];
    let via_r = fr_route::via_radius(board, ctx.via_padstack, ctx.grid.pitch);
    let via_exact_r = fr_route::via_radius(board, ctx.via_padstack, 0).max(1);

    // Multi-pass: retry not-yet-completed nets. Later passes can succeed because the set
    // of committed traces differs, opening gaps; passes stop when one yields no progress
    // or the time budget runs out. (A full rip-up-and-reroute is future work; this order
    // + retry already lifts completion notably.)
    const MAX_PASSES: usize = 4;
    for pass in 0..MAX_PASSES {
        let mut progressed = false;
        let mut still_incomplete = 0usize;
        for &net_id in &order {
            if completed[net_id] {
                continue;
            }
            if budget.map(|b| start.elapsed() >= b).unwrap_or(false) {
                break;
            }
            let pin_pts = net_pin_points(board, net_id);
            if pin_pts.len() < 2 {
                completed[net_id] = true;
                continue;
            }
            // Route all MST edges of this net; only commit if ALL succeed, so a partial
            // net doesn't leave dangling stubs blocking the retry.
            let mut produced = Vec::new();
            let mut ok = true;
            let validator = EdgeValidator {
                index: &index,
                half: ctx.width / 2,
                clearance: ctx.clearance,
            };
            for (ai, bi) in mst_edges(&pin_pts) {
                match route_connection(
                    board, ctx.grid, &obs, net_id as u32,
                    pin_pts[ai], pin_pts[bi],
                    ctx.width, Some(ctx.via_padstack), &ctx.costs, ctx.max_expansions,
                    Some(&validator),
                ) {
                    Some(c) => produced.push(c),
                    None => { ok = false; break; }
                }
            }
            // Commit a net ONLY if all its MST edges routed. A partially-routed net is
            // discarded entirely (no dangling/isolated stubs on the board) and retried
            // next pass. This is the "drop partial nets" policy.
            if ok {
                for c in &produced {
                    for t in &c.traces {
                        obs.stamp_trace(t.layer, &t.corners, t.width, net_id as u32);
                        index.add_trace(t.layer, &t.corners, t.width, net_id as u32);
                    }
                    for v in &c.vias {
                        obs.stamp_via(v.location, via_r, net_id as u32);
                        index.add_via(0, ctx.grid.layers - 1, v.location, via_exact_r, net_id as u32);
                    }
                }
                report.connections_routed += produced.len();
                for c in produced {
                    board.traces.extend(c.traces);
                    board.vias.extend(c.vias);
                }
                completed[net_id] = true;
                progressed = true;
            } else {
                still_incomplete += 1;
            }
        }
        if !progressed || still_incomplete == 0 {
            let _ = pass;
            break;
        }
    }

    report.nets_completed = completed.iter().filter(|c| **c).count();
    report.passes = MAX_PASSES;
    report.unrouted_nets = (0..net_count).filter(|&i| !completed[i]).collect();
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
/// Public: the MST ratsnest edges of a net as point pairs, for GUI display of unrouted
/// connections. Returns the same edges the router would attempt.
pub fn net_ratsnest(board: &Board, net_id: usize) -> Vec<(Point, Point)> {
    let pts = net_pin_points(board, net_id);
    mst_edges(&pts).into_iter().map(|(a, b)| (pts[a], pts[b])).collect()
}

pub fn net_pin_points(board: &Board, net_id: usize) -> Vec<Point> {
    let mut pts: Vec<Point> = board.pins_of_net(net_id).map(|p| p.location).collect();
    pts.sort_by_key(|p| (p.x, p.y));
    pts.dedup();
    pts
}

fn choose_pitch(board: &Board, bounds: fr_geometry::IntBox) -> i64 {
    // The pitch must be FINE ENOUGH to represent clearance: if one grid step is larger
    // than (max_pad_radius + trace_half_width + clearance), a trace routed one node away
    // from a pad still overlaps it -> trace-to-pad shorts. So cap the pitch at roughly
    // that spacing. We also keep a floor so the grid doesn't explode on huge boards.
    let half = board.rules.default_width / 2;
    let clr = board.rules.default_clearance;
    let max_pad_r = board
        .padstacks
        .iter_radii()
        .max()
        .unwrap_or(board.rules.default_width);
    // a trace centerline must be able to sit > (pad_r + half + clr) from a pad center;
    // pick pitch so one step is at most that spacing (use ~half of it for headroom).
    let feature = (max_pad_r + half + clr).max(1);
    let want = (feature / 2).max(board.resolution.per_unit); // >= ~1 mil
    // floor: don't let the grid exceed ~2500 cells on the long side (perf).
    let span = bounds.width().max(bounds.height()).max(1);
    let floor = span / 1500;
    want.max(floor).max(1)
}

/// Public wrapper: ensure the board has a routing-via padstack and return its index
/// (used by the interactive router to build vias).
pub fn ensure_via_padstack_pub(board: &mut Board) -> usize {
    ensure_via_padstack(board)
}

fn ensure_via_padstack(board: &mut Board) -> usize {
    let layer_count = board.layer_count().max(1);
    let spans_all = |p: &Padstack| {
        !p.is_empty() && p.from_layer() == Some(0) && p.to_layer() == Some(layer_count - 1)
    };
    // Prefer a real routing-via padstack: one whose NAME contains "via" and that spans
    // the full stack. Using an arbitrary layer-0 pad (e.g. a through-hole component pad)
    // as the via produces wrong-looking vias on import in Altium.
    if let Some(i) = (0..board.padstacks.len()).find(|&i| {
        board.padstacks.get(i).map(|p| {
            p.name.to_ascii_lowercase().contains("via") && spans_all(p)
        }).unwrap_or(false)
    }) {
        return i;
    }
    // Next best: any padstack that spans the full layer stack.
    if let Some(i) = (0..board.padstacks.len())
        .find(|&i| board.padstacks.get(i).map(spans_all).unwrap_or(false))
    {
        return i;
    }
    // Fallback: synthesize a through via.
    board.padstacks.add(Padstack {
        name: "Via_fr_rs".into(),
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
    fn prefers_named_via_padstack() {
        use fr_board::{PadShape, Padstack};
        let mut b = two_pin_board();
        // a real routing-via padstack spanning both layers, plus a decoy through-pad
        b.padstacks.add(Padstack {
            name: "MyRoutingVia".into(),
            shapes: vec![Some(PadShape::Circle { radius: 110_000 }), Some(PadShape::Circle { radius: 110_000 })],
            drillable: true,
        });
        let idx = ensure_via_padstack(&mut b);
        assert_eq!(b.padstacks.get(idx).unwrap().name, "MyRoutingVia",
            "should pick the padstack named like a via");
    }

    #[test]
    fn synthesizes_via_when_none_present() {
        let mut b = two_pin_board(); // no full-span padstacks
        let idx = ensure_via_padstack(&mut b);
        let ps = b.padstacks.get(idx).unwrap();
        assert!(!ps.is_empty() && ps.from_layer() == Some(0));
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
        let report = route_board(&mut b, &RouteOptions { max_time_secs: 0, threads: 0, seed: 1, ..Default::default() });
        assert_eq!(report.nets_completed, 1, "parallel path should connect the net");
        assert!(!b.traces.is_empty());
    }

    #[test]
    fn parallel_is_deterministic() {
        let mut b1 = two_pin_board();
        let mut b2 = two_pin_board();
        route_board(&mut b1, &RouteOptions { max_time_secs: 0, threads: 4, seed: 1, ..Default::default() });
        route_board(&mut b2, &RouteOptions { max_time_secs: 0, threads: 4, seed: 1, ..Default::default() });
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

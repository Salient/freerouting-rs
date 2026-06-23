//! Weighted A* over the routing grid, with via (layer-change) moves.
//!
//! This is the grid-era realization of the spec's weighted-A* maze search: a binary
//! min-heap frontier ordered by f = g + h, an admissible Manhattan+via heuristic, and
//! a closed set. It mirrors the structure the room/door search will use, so swapping
//! the neighbour generator later is localized. Costs and the search are deterministic.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::grid::{Grid, Node};
use crate::obstacles::ObstacleMap;
use fr_geometry::Point;
use fr_spatial::ObstacleIndex;

/// Exact-geometry gate on an A* in-plane edge: the trace segment swept between two node
/// centers must keep `clearance` from every different-net copper shape in `index`. This
/// is what the coarse grid's node-center passability check misses — a segment (especially
/// a diagonal) can run between two passable cells yet clip a pad larger than the pitch.
/// Supplying this validator makes the search reject such edges, structurally removing the
/// trace-to-pad shorts the grid alone cannot avoid.
#[derive(Clone, Copy)]
pub struct EdgeValidator<'a> {
    pub index: &'a ObstacleIndex,
    /// Half the routed trace width (board units).
    pub half: i64,
    /// Required copper clearance (board units).
    pub clearance: i64,
}

impl EdgeValidator<'_> {
    /// True if the trace segment from `a` to `b` on `layer` is clear for `net`.
    fn edge_clear(&self, layer: u32, a: Point, b: Point, net: u32) -> bool {
        self.index.segment_is_clear(layer as usize, a, b, self.half, net, self.clearance)
    }
}

/// Cost parameters for the search (board-unit scaled).
#[derive(Clone, Copy, Debug)]
pub struct Costs {
    /// Cost of moving one cell orthogonally (~ the pitch).
    pub step: i64,
    /// Cost of moving one cell diagonally (~ pitch * sqrt(2)).
    pub diag: i64,
    /// Cost of a via (layer change).
    pub via: i64,
}

impl Costs {
    pub fn for_grid(g: &Grid, via_cost: i64) -> Costs {
        Costs { step: g.pitch, diag: (g.pitch as f64 * 1.41421356).round() as i64, via: via_cost }
    }
}

#[derive(Clone, Copy, Eq, PartialEq)]
struct HeapItem {
    f: i64,
    g: i64,
    node: Node,
}

impl Ord for HeapItem {
    fn cmp(&self, other: &Self) -> Ordering {
        // min-heap: reverse on f, then g, then a stable node ordering for determinism.
        other
            .f
            .cmp(&self.f)
            .then_with(|| other.g.cmp(&self.g))
            .then_with(|| node_key(self.node).cmp(&node_key(other.node)))
    }
}
impl PartialOrd for HeapItem {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

fn node_key(n: Node) -> (u32, i32, i32) {
    (n.layer, n.col, n.row)
}

/// 8 in-plane neighbours + 2 layer changes.
const DIRS: [(i32, i32); 8] = [
    (1, 0), (-1, 0), (0, 1), (0, -1),
    (1, 1), (1, -1), (-1, 1), (-1, -1),
];

/// Admissible heuristic: octile distance in-plane + a via if layers differ.
fn heuristic(a: Node, b: Node, c: &Costs) -> i64 {
    let dx = (a.col - b.col).unsigned_abs() as i64;
    let dy = (a.row - b.row).unsigned_abs() as i64;
    let (lo, hi) = if dx < dy { (dx, dy) } else { (dy, dx) };
    let plane = lo * c.diag + (hi - lo) * c.step;
    let via = if a.layer != b.layer { c.via } else { 0 };
    plane + via
}

/// Find a least-cost path from any node in `starts` to any node in `goals`, staying on
/// cells passable for `net`. Returns the node path start..goal, or None if unreachable.
/// `max_expansions` bounds the work (so a hard net cannot stall the whole board).
///
/// This wrapper allocates a fresh scratch buffer per call (fine for tests / one-off use).
/// The engine's hot loop should reuse an `AStarScratch` across calls via `search_scratch`
/// to avoid re-allocating + zeroing ~15 MB of dense arrays on every search.
#[allow(clippy::too_many_arguments)]
pub fn search(
    grid: &Grid,
    obs: &ObstacleMap,
    net: u32,
    starts: &[Node],
    goals: &[Node],
    costs: &Costs,
    max_expansions: usize,
    validator: Option<&EdgeValidator>,
) -> Option<Vec<Node>> {
    let mut scratch = AStarScratch::new(grid);
    search_scratch(grid, obs, net, starts, goals, costs, max_expansions, validator, &mut scratch)
}

/// Reusable A* working memory: the dense per-node arrays, kept across searches so they are
/// allocated once per routing run instead of once per connection. A monotonically
/// increasing `generation` stamp marks which cells are "live" this search, so we never
/// have to zero the whole array — a cell is unvisited unless its stored generation equals
/// the current one. `touched` records the cells written this search so `goal_flag` can be
/// cleared cheaply.
pub struct AStarScratch {
    n_nodes: usize,
    generation: u32,
    /// (generation, g_score) per node; stale if gen != current.
    gen_of: Vec<u32>,
    g_score: Vec<i64>,
    came_from: Vec<u32>,
    goal_flag: Vec<bool>,
    touched_goals: Vec<usize>,
}

impl AStarScratch {
    pub fn new(grid: &Grid) -> AStarScratch {
        let n = (grid.cols * grid.rows * grid.layers as i64) as usize;
        AStarScratch {
            n_nodes: n,
            generation: 0,
            gen_of: vec![0; n],
            g_score: vec![i64::MAX; n],
            came_from: vec![u32::MAX; n],
            goal_flag: vec![false; n],
            touched_goals: Vec::new(),
        }
    }
}

/// A*-with-reusable-scratch. Behaviour is identical to `search`; only the working memory
/// is supplied by the caller. The scratch must have been built for a grid of the same
/// node count.
#[allow(clippy::too_many_arguments)]
pub fn search_scratch(
    grid: &Grid,
    obs: &ObstacleMap,
    net: u32,
    starts: &[Node],
    goals: &[Node],
    costs: &Costs,
    max_expansions: usize,
    validator: Option<&EdgeValidator>,
    scratch: &mut AStarScratch,
) -> Option<Vec<Node>> {
    if starts.is_empty() || goals.is_empty() {
        return None;
    }
    let n_nodes = (grid.cols * grid.rows * grid.layers as i64) as usize;
    debug_assert_eq!(n_nodes, scratch.n_nodes, "scratch built for a different grid");
    let id = |n: Node| -> usize {
        ((n.layer as i64 * grid.cols + n.col as i64) * grid.rows + n.row as i64) as usize
    };

    // bump the generation; a node is "seen" this search iff gen_of[id] == generation.
    scratch.generation = scratch.generation.wrapping_add(1);
    let gen = scratch.generation;
    // clear only the goal flags we set last time.
    for &i in &scratch.touched_goals {
        scratch.goal_flag[i] = false;
    }
    scratch.touched_goals.clear();

    for &gnode in goals {
        if grid.in_bounds(gnode) {
            let i = id(gnode);
            scratch.goal_flag[i] = true;
            scratch.touched_goals.push(i);
        }
    }
    let href = goals[0];

    let mut open = BinaryHeap::new();
    for &s in starts {
        if !obs.passable(s, net) {
            continue;
        }
        let h = heuristic(s, href, costs);
        let i = id(s);
        scratch.gen_of[i] = gen;
        scratch.g_score[i] = 0;
        scratch.came_from[i] = u32::MAX;
        open.push(HeapItem { f: h, g: 0, node: s });
    }

    // current best g for a node, treating stale (old-generation) entries as infinity.
    let g_at = |scratch: &AStarScratch, i: usize| -> i64 {
        if scratch.gen_of[i] == gen { scratch.g_score[i] } else { i64::MAX }
    };

    let mut expansions = 0usize;
    while let Some(HeapItem { f: _, g, node }) = open.pop() {
        let nid = id(node);
        if g > g_at(scratch, nid) {
            continue; // stale
        }
        if scratch.goal_flag[nid] {
            return Some(reconstruct_gen(scratch, node, grid, gen));
        }
        expansions += 1;
        if expansions > max_expansions {
            return None;
        }
        // in-plane neighbours
        for (dc, dr) in DIRS {
            let nb = Node { layer: node.layer, col: node.col + dc, row: node.row + dr };
            if !obs.passable(nb, net) {
                continue;
            }
            if let Some(v) = validator {
                if !v.edge_clear(node.layer, grid.point_of(node), grid.point_of(nb), net) {
                    continue;
                }
            }
            let move_cost = if dc != 0 && dr != 0 { costs.diag } else { costs.step };
            relax_gen(node, nb, g + move_cost, href, costs, &id, gen, scratch, &mut open);
        }
        // layer changes (via) at the same col/row
        for layer in 0..grid.layers as u32 {
            if layer == node.layer {
                continue;
            }
            let nb = Node { layer, col: node.col, row: node.row };
            if !obs.passable(nb, net) {
                continue;
            }
            relax_gen(node, nb, g + costs.via, href, costs, &id, gen, scratch, &mut open);
        }
    }
    None
}

#[allow(clippy::too_many_arguments)]
fn relax_gen(
    from: Node,
    to: Node,
    tentative_g: i64,
    href: Node,
    costs: &Costs,
    id: &impl Fn(Node) -> usize,
    gen: u32,
    scratch: &mut AStarScratch,
    open: &mut BinaryHeap<HeapItem>,
) {
    let tid = id(to);
    let cur = if scratch.gen_of[tid] == gen { scratch.g_score[tid] } else { i64::MAX };
    if tentative_g < cur {
        scratch.gen_of[tid] = gen;
        scratch.g_score[tid] = tentative_g;
        scratch.came_from[tid] = id(from) as u32;
        let f = tentative_g + heuristic(to, href, costs);
        open.push(HeapItem { f, g: tentative_g, node: to });
    }
}

fn reconstruct_gen(scratch: &AStarScratch, goal: Node, grid: &Grid, gen: u32) -> Vec<Node> {
    let decode = |fid: u32| -> Node {
        let v = fid as i64;
        let row = v % grid.rows;
        let col = (v / grid.rows) % grid.cols;
        let layer = v / (grid.rows * grid.cols);
        Node { layer: layer as u32, col: col as i32, row: row as i32 }
    };
    let id = |n: Node| -> usize {
        ((n.layer as i64 * grid.cols + n.col as i64) * grid.rows + n.row as i64) as usize
    };
    let mut path = vec![goal];
    let mut cur = goal;
    loop {
        let i = id(cur);
        if scratch.gen_of[i] != gen {
            break;
        }
        let p = scratch.came_from[i];
        if p == u32::MAX {
            break;
        }
        cur = decode(p);
        path.push(cur);
    }
    path.reverse();
    path
}

#[cfg(test)]
mod tests {
    use super::*;
    use fr_board::{Board, Layer, LayerStack, Pin, PadShape, Padstack, Resolution, Unit};
    use fr_geometry::{IntBox, Point};

    fn one_layer_board() -> Board {
        let mut b = Board::new("t".into(), Resolution::new(Unit::Mil, 10000));
        b.layers = LayerStack::new(vec![Layer { name: "Top".into(), index: 0, is_signal: true, preferred: None }]);
        b
    }

    #[test]
    fn finds_straight_path_in_empty_space() {
        let board = one_layer_board();
        let grid = Grid::new(IntBox::new(0, 0, 1_000_000, 1_000_000), 50_000, 1);
        let obs = ObstacleMap::build(&board, &grid);
        let start = grid.node_at(0, Point::new(100_000, 500_000));
        let goal = grid.node_at(0, Point::new(900_000, 500_000));
        let costs = Costs::for_grid(&grid, 500_000);
        let path = search(&grid, &obs, 0, &[start], &[goal], &costs, 100_000, None).expect("path exists");
        assert_eq!(*path.first().unwrap(), start);
        assert_eq!(*path.last().unwrap(), goal);
    }

    #[test]
    fn routes_around_a_blocking_pin() {
        let mut board = one_layer_board();
        let mut shapes = vec![Some(PadShape::Circle { radius: 150_000 })];
        let pad = board.padstacks.add(Padstack { name: "B".into(), shapes: std::mem::take(&mut shapes), drillable: false });
        // blocker for a DIFFERENT net (net 1) sits in the middle
        board.pins.push(Pin { component: "X".into(), name: "1".into(), padstack: pad, location: Point::new(500_000, 500_000), net: Some(1), rotation: 0.0, front: true });
        let grid = Grid::new(IntBox::new(0, 0, 1_000_000, 1_000_000), 50_000, 1);
        let obs = ObstacleMap::build(&board, &grid);
        let start = grid.node_at(0, Point::new(100_000, 500_000));
        let goal = grid.node_at(0, Point::new(900_000, 500_000));
        let costs = Costs::for_grid(&grid, 500_000);
        // routing net 0 must detour around net 1's pin
        let path = search(&grid, &obs, 0, &[start], &[goal], &costs, 1_000_000, None).expect("detour path");
        // no node of the path is the blocked center
        let center = grid.node_at(0, Point::new(500_000, 500_000));
        assert!(!path.contains(&center));
    }

    #[test]
    fn deterministic_same_seed_same_path() {
        let board = one_layer_board();
        let grid = Grid::new(IntBox::new(0, 0, 1_000_000, 1_000_000), 50_000, 1);
        let obs = ObstacleMap::build(&board, &grid);
        let start = grid.node_at(0, Point::new(100_000, 100_000));
        let goal = grid.node_at(0, Point::new(900_000, 900_000));
        let costs = Costs::for_grid(&grid, 500_000);
        let p1 = search(&grid, &obs, 0, &[start], &[goal], &costs, 1_000_000, None).unwrap();
        let p2 = search(&grid, &obs, 0, &[start], &[goal], &costs, 1_000_000, None).unwrap();
        assert_eq!(p1, p2, "search must be deterministic");
    }
}

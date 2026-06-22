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
pub fn search(
    grid: &Grid,
    obs: &ObstacleMap,
    net: u32,
    starts: &[Node],
    goals: &[Node],
    costs: &Costs,
    max_expansions: usize,
) -> Option<Vec<Node>> {
    if starts.is_empty() || goals.is_empty() {
        return None;
    }
    // Dense arrays indexed by a flat node id: far faster than hashing on a big grid.
    let n_nodes = (grid.cols * grid.rows * grid.layers as i64) as usize;
    let id = |n: Node| -> usize {
        ((n.layer as i64 * grid.cols + n.col as i64) * grid.rows + n.row as i64) as usize
    };
    let mut g_score: Vec<i64> = vec![i64::MAX; n_nodes];
    let mut came_from: Vec<u32> = vec![u32::MAX; n_nodes];

    let mut goal_flag: Vec<bool> = vec![false; n_nodes];
    for &gnode in goals {
        if grid.in_bounds(gnode) {
            goal_flag[id(gnode)] = true;
        }
    }
    let href = goals[0];

    let mut open = BinaryHeap::new();
    for &s in starts {
        if !obs.passable(s, net) {
            continue;
        }
        let h = heuristic(s, href, costs);
        g_score[id(s)] = 0;
        open.push(HeapItem { f: h, g: 0, node: s });
    }

    let mut expansions = 0usize;
    while let Some(HeapItem { f: _, g, node }) = open.pop() {
        let nid = id(node);
        if g > g_score[nid] {
            continue; // stale
        }
        if goal_flag[nid] {
            return Some(reconstruct(&came_from, node, grid));
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
            let move_cost = if dc != 0 && dr != 0 { costs.diag } else { costs.step };
            relax(node, nb, g + move_cost, href, costs, &id, &mut g_score, &mut came_from, &mut open);
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
            relax(node, nb, g + costs.via, href, costs, &id, &mut g_score, &mut came_from, &mut open);
        }
    }
    None
}

#[allow(clippy::too_many_arguments)]
fn relax(
    from: Node,
    to: Node,
    tentative_g: i64,
    href: Node,
    costs: &Costs,
    id: &impl Fn(Node) -> usize,
    g_score: &mut [i64],
    came_from: &mut [u32],
    open: &mut BinaryHeap<HeapItem>,
) {
    let tid = id(to);
    if tentative_g < g_score[tid] {
        g_score[tid] = tentative_g;
        came_from[tid] = id(from) as u32;
        let f = tentative_g + heuristic(to, href, costs);
        open.push(HeapItem { f, g: tentative_g, node: to });
    }
}

fn reconstruct(came_from: &[u32], goal: Node, grid: &Grid) -> Vec<Node> {
    // decode a flat id back into a Node
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
        let p = came_from[id(cur)];
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
        let path = search(&grid, &obs, 0, &[start], &[goal], &costs, 100_000).expect("path exists");
        assert_eq!(*path.first().unwrap(), start);
        assert_eq!(*path.last().unwrap(), goal);
    }

    #[test]
    fn routes_around_a_blocking_pin() {
        let mut board = one_layer_board();
        let mut shapes = vec![Some(PadShape::Circle { radius: 150_000 })];
        let pad = board.padstacks.add(Padstack { name: "B".into(), shapes: std::mem::take(&mut shapes), drillable: false });
        // blocker for a DIFFERENT net (net 1) sits in the middle
        board.pins.push(Pin { component: "X".into(), name: "1".into(), padstack: pad, location: Point::new(500_000, 500_000), net: Some(1) });
        let grid = Grid::new(IntBox::new(0, 0, 1_000_000, 1_000_000), 50_000, 1);
        let obs = ObstacleMap::build(&board, &grid);
        let start = grid.node_at(0, Point::new(100_000, 500_000));
        let goal = grid.node_at(0, Point::new(900_000, 500_000));
        let costs = Costs::for_grid(&grid, 500_000);
        // routing net 0 must detour around net 1's pin
        let path = search(&grid, &obs, 0, &[start], &[goal], &costs, 1_000_000).expect("detour path");
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
        let p1 = search(&grid, &obs, 0, &[start], &[goal], &costs, 1_000_000).unwrap();
        let p2 = search(&grid, &obs, 0, &[start], &[goal], &costs, 1_000_000).unwrap();
        assert_eq!(p1, p2, "search must be deterministic");
    }
}

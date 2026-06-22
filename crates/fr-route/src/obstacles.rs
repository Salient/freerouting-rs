//! Per-layer obstacle occupancy derived from the board, used by the A* search to keep
//! traces clear of pads, other nets' pins, and already-routed traces.
//!
//! Backed by a coarse grid bitmap (one bit per (layer,col,row)). A cell is blocked for
//! the net being routed if it is within (clearance + half-width) of an obstacle that
//! does NOT belong to that net. Pins of the net being routed are passable (we must
//! reach them). This is the grid-era stand-in for the exact room/door clearance model.

use crate::grid::{Grid, Node};
use fr_board::Board;
use fr_geometry::Point;

pub struct ObstacleMap<'a> {
    grid: &'a Grid,
    /// blocked[layer][col*rows+row] = generic obstacle (any net's copper/pad).
    blocked: Vec<Vec<bool>>,
    /// owner net id per cell, if the cell is occupied by a specific net (so same-net
    /// cells are passable). usize::MAX = no specific owner / multi.
    owner: Vec<Vec<u32>>,
}

const NO_OWNER: u32 = u32::MAX;

impl<'a> ObstacleMap<'a> {
    pub fn build(board: &Board, grid: &'a Grid) -> ObstacleMap<'a> {
        let per_layer = (grid.cols * grid.rows) as usize;
        let mut map = ObstacleMap {
            grid,
            blocked: vec![vec![false; per_layer]; grid.layers],
            owner: vec![vec![NO_OWNER; per_layer]; grid.layers],
        };

        // Stamp pin footprints. A pin blocks the cells around its location on the layers
        // its padstack covers, tagged with its net (so its own net can route into it).
        for pin in &board.pins {
            let Some(ps) = board.padstacks.get(pin.padstack) else { continue };
            if ps.is_empty() {
                continue; // shapeless padstack: no copper, skip
            }
            let net = pin.net.map(|n| n as u32).unwrap_or(NO_OWNER);
            // approximate footprint radius: the max circle radius across layers, else pitch
            let radius = ps
                .shapes
                .iter()
                .filter_map(|s| match s {
                    Some(fr_board::PadShape::Circle { radius }) => Some(*radius),
                    _ => None,
                })
                .max()
                .unwrap_or(grid.pitch);
            let (lo, hi) = (ps.from_layer().unwrap_or(0), ps.to_layer().unwrap_or(grid.layers - 1));
            for layer in lo..=hi.min(grid.layers - 1) {
                map.stamp_disc(layer, pin.location, radius, net);
            }
        }

        // Existing traces/vias on the board also block (tagged by their net).
        for t in &board.traces {
            let net = t.net.map(|n| n as u32).unwrap_or(NO_OWNER);
            let half = t.width / 2;
            for w in t.corners.windows(2) {
                map.stamp_segment(t.layer.min(grid.layers - 1), w[0], w[1], half, net);
            }
        }
        for v in &board.vias {
            let net = v.net.map(|n| n as u32).unwrap_or(NO_OWNER);
            let r = board
                .padstacks
                .get(v.padstack)
                .and_then(|p| p.shapes.iter().filter_map(|s| match s {
                    Some(fr_board::PadShape::Circle { radius }) => Some(*radius),
                    _ => None,
                }).max())
                .unwrap_or(grid.pitch);
            for layer in 0..grid.layers {
                map.stamp_disc(layer, v.location, r, net);
            }
        }

        map
    }

    fn idx(&self, n: Node) -> usize {
        (n.col as i64 * self.grid.rows + n.row as i64) as usize
    }

    fn stamp_disc(&mut self, layer: usize, center: Point, radius: i64, net: u32) {
        if layer >= self.grid.layers {
            return;
        }
        let r_cells = (radius / self.grid.pitch).max(0) + 1;
        let c = self.grid.node_at(layer, center);
        for dc in -r_cells..=r_cells {
            for dr in -r_cells..=r_cells {
                let n = Node { layer: layer as u32, col: c.col + dc as i32, row: c.row + dr as i32 };
                if self.grid.in_bounds(n) {
                    let i = self.idx(n);
                    self.blocked[layer][i] = true;
                    // first owner wins; conflicting owners -> generic (NO_OWNER stays blocked)
                    if self.owner[layer][i] == NO_OWNER {
                        self.owner[layer][i] = net;
                    } else if self.owner[layer][i] != net {
                        self.owner[layer][i] = NO_OWNER;
                    }
                }
            }
        }
    }

    fn stamp_segment(&mut self, layer: usize, a: Point, b: Point, half: i64, net: u32) {
        // sample along the segment at pitch intervals, stamping a disc at each
        let steps = (a.to_float().distance(b.to_float()) / self.grid.pitch as f64).ceil() as i64 + 1;
        for s in 0..=steps {
            let t = s as f64 / steps as f64;
            let p = Point::new(
                a.x + ((b.x - a.x) as f64 * t).round() as i64,
                a.y + ((b.y - a.y) as f64 * t).round() as i64,
            );
            self.stamp_disc(layer, p, half, net);
        }
    }

    /// Is node `n` passable for the net `net` (own-net cells are passable)?
    pub fn passable(&self, n: Node, net: u32) -> bool {
        if !self.grid.in_bounds(n) {
            return false;
        }
        let layer = n.layer as usize;
        let i = self.idx(n);
        if !self.blocked[layer][i] {
            return true;
        }
        self.owner[layer][i] == net
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fr_board::{Pin, Resolution, Unit};
    use fr_geometry::IntBox;

    #[test]
    fn pin_blocks_other_nets_but_not_own() {
        let mut board = Board::new("t".into(), Resolution::new(Unit::Mil, 10000));
        // one layer
        board.layers = fr_board::LayerStack::new(vec![fr_board::Layer {
            name: "Top".into(), index: 0, is_signal: true, preferred: None,
        }]);
        let mut shapes = vec![Some(fr_board::PadShape::Circle { radius: 50_000 })];
        let pad = board.padstacks.add(fr_board::Padstack { name: "P".into(), shapes: std::mem::take(&mut shapes), drillable: false });
        board.pins.push(Pin { component: "U1".into(), name: "1".into(), padstack: pad, location: Point::new(500_000, 500_000), net: Some(0) });

        let grid = Grid::new(IntBox::new(0, 0, 1_000_000, 1_000_000), 50_000, 1);
        let map = ObstacleMap::build(&board, &grid);
        let at_pin = grid.node_at(0, Point::new(500_000, 500_000));
        assert!(map.passable(at_pin, 0), "own net 0 may enter its pin");
        assert!(!map.passable(at_pin, 1), "other net 1 is blocked by the pin");
    }
}

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
    /// Design clearance in board units, added to every stamped radius so that two
    /// different-net traces cannot be placed in abutting cells (which would short).
    clearance: i64,
}

const NO_OWNER: u32 = u32::MAX;

impl<'a> ObstacleMap<'a> {
    pub fn build(board: &Board, grid: &'a Grid) -> ObstacleMap<'a> {
        Self::build_with_clearance(board, grid, board.rules.default_clearance)
    }

    pub fn build_with_clearance(board: &Board, grid: &'a Grid, clearance: i64) -> ObstacleMap<'a> {
        let per_layer = (grid.cols * grid.rows) as usize;
        let mut map = ObstacleMap {
            grid,
            blocked: vec![vec![false; per_layer]; grid.layers],
            owner: vec![vec![NO_OWNER; per_layer]; grid.layers],
            clearance: clearance.max(0),
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
            map.stamp_trace(t.layer, &t.corners, t.width, net);
        }
        for v in &board.vias {
            let net = v.net.map(|n| n as u32).unwrap_or(NO_OWNER);
            let r = via_radius(board, v.padstack, grid.pitch);
            map.stamp_via(v.location, r, net);
        }

        map
    }

    /// Stamp a routed trace (full rasterized path, width + clearance) tagged with `net`.
    /// Used to add a net's geometry to the map incrementally as routing progresses, so
    /// the next net's search sees it and cannot overlap it (the fix for shorting).
    pub fn stamp_trace(&mut self, layer: usize, corners: &[Point], width: i64, net: u32) {
        // Block this trace's half-width PLUS another half-width: the A* routes a new
        // trace along node centerlines, so to keep that new (same-width) trace's EDGE at
        // least `clearance` away from THIS trace's edge, the blocked footprint must
        // reserve both half-widths (stamp_disc adds the clearance on top). Without the
        // extra half-width, two centerlines could sit exactly `clearance` apart and their
        // copper would touch.
        let reserve = width; // = half (this) + half (future trace)
        let layer = layer.min(self.grid.layers - 1);
        for w in corners.windows(2) {
            self.stamp_segment(layer, w[0], w[1], reserve, net);
        }
    }

    /// Stamp a via (a disc on every layer) tagged with `net`.
    pub fn stamp_via(&mut self, center: Point, radius: i64, net: u32) {
        for layer in 0..self.grid.layers {
            self.stamp_disc(layer, center, radius, net);
        }
    }

    fn idx(&self, n: Node) -> usize {
        (n.col as i64 * self.grid.rows + n.row as i64) as usize
    }

    fn stamp_disc(&mut self, layer: usize, center: Point, radius: i64, net: u32) {
        if layer >= self.grid.layers {
            return;
        }
        // Add the design clearance so a different-net trace must keep its distance; the
        // +1 rounds the footprint out to whole cells. This is what prevents adjacent
        // parallel traces of different nets from touching (shorting).
        let r_cells = ((radius + self.clearance) / self.grid.pitch).max(0) + 1;
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
        // Sample along the segment at HALF-pitch intervals so a diagonal segment cannot
        // skip a grid cell between samples (which would leave a gap another net could
        // route through, causing a touch/short).
        let steps = (2.0 * a.to_float().distance(b.to_float()) / self.grid.pitch as f64).ceil() as i64 + 1;
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

/// The footprint radius of a via's padstack (max circle radius across layers), or the
/// grid pitch as a fallback.
pub fn via_radius(board: &Board, padstack: usize, fallback: i64) -> i64 {
    board
        .padstacks
        .get(padstack)
        .and_then(|p| {
            p.shapes
                .iter()
                .filter_map(|s| match s {
                    Some(fr_board::PadShape::Circle { radius }) => Some(*radius),
                    _ => None,
                })
                .max()
        })
        .unwrap_or(fallback)
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

    #[test]
    fn stamped_trace_blocks_other_nets_with_clearance() {
        let mut board = Board::new("t".into(), Resolution::new(Unit::Mil, 10000));
        board.layers = fr_board::LayerStack::new(vec![fr_board::Layer {
            name: "Top".into(), index: 0, is_signal: true, preferred: None,
        }]);
        let grid = Grid::new(IntBox::new(0, 0, 2_000_000, 2_000_000), 20_000, 1);
        let mut map = ObstacleMap::build_with_clearance(&board, &grid, 80_000);
        // a horizontal net-0 trace across the middle
        let corners = [Point::new(200_000, 1_000_000), Point::new(1_800_000, 1_000_000)];
        map.stamp_trace(0, &corners, 100_000, 0);

        // a cell right on the trace is blocked for net 1 but passable for net 0
        let on = grid.node_at(0, Point::new(1_000_000, 1_000_000));
        assert!(map.passable(on, 0), "own net may follow its own trace");
        assert!(!map.passable(on, 1), "other net is blocked on the trace");
        // a cell within (half-width + clearance) is also blocked for net 1 (no touch)
        let near = grid.node_at(0, Point::new(1_000_000, 1_060_000));
        assert!(!map.passable(near, 1), "other net is blocked within clearance of the trace");
        // far away is free
        let far = grid.node_at(0, Point::new(1_000_000, 1_500_000));
        assert!(map.passable(far, 1), "other net is free far from the trace");
    }
}

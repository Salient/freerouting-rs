//! Routing search space.
//!
//! The spec's end goal is the free-angle expansion-room/door model. As the foundation
//! (Phase 5), this module provides a coarse uniform grid abstraction over the board
//! extent that the A* search runs on: it is simple, deterministic, and enough to route
//! real nets end-to-end so the whole pipeline (DSN -> route -> RTE -> Altium) is
//! exercised. The room/door model replaces the grid's neighbour generation later
//! without changing the A* driver or the engine API.

use fr_geometry::{IntBox, Point};

/// A uniform routing grid over the board bounding box. Cells are `pitch` board units.
#[derive(Clone, Debug)]
pub struct Grid {
    pub origin: Point,
    pub pitch: i64,
    pub cols: i64,
    pub rows: i64,
    pub layers: usize,
}

/// A grid node: (layer, col, row).
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct Node {
    pub layer: u32,
    pub col: i32,
    pub row: i32,
}

impl Grid {
    /// Build a grid covering `bounds` (with a margin) at the given pitch and layer count.
    pub fn new(bounds: IntBox, pitch: i64, layers: usize) -> Grid {
        let pitch = pitch.max(1);
        let margin = pitch * 2;
        let ll = Point::new(bounds.ll.x - margin, bounds.ll.y - margin);
        let w = bounds.width() + 2 * margin;
        let h = bounds.height() + 2 * margin;
        Grid {
            origin: ll,
            pitch,
            cols: (w / pitch).max(1) + 1,
            rows: (h / pitch).max(1) + 1,
            layers: layers.max(1),
        }
    }

    /// Snap a board point to the nearest grid node on a layer.
    pub fn node_at(&self, layer: usize, p: Point) -> Node {
        let col = ((p.x - self.origin.x) as f64 / self.pitch as f64).round() as i32;
        let row = ((p.y - self.origin.y) as f64 / self.pitch as f64).round() as i32;
        Node {
            layer: layer as u32,
            col: col.clamp(0, (self.cols - 1) as i32),
            row: row.clamp(0, (self.rows - 1) as i32),
        }
    }

    /// Board-unit center of a grid node.
    pub fn point_of(&self, n: Node) -> Point {
        Point::new(
            self.origin.x + n.col as i64 * self.pitch,
            self.origin.y + n.row as i64 * self.pitch,
        )
    }

    pub fn in_bounds(&self, n: Node) -> bool {
        n.col >= 0
            && (n.col as i64) < self.cols
            && n.row >= 0
            && (n.row as i64) < self.rows
            && (n.layer as usize) < self.layers
    }

    /// Total node count (for sanity/bounds).
    pub fn node_count(&self) -> i64 {
        self.cols * self.rows * self.layers as i64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn snap_and_unsnap_roundtrip() {
        let g = Grid::new(IntBox::new(0, 0, 1000, 1000), 100, 2);
        let n = g.node_at(0, Point::new(503, 397));
        // nearest node center should be within half a pitch
        let p = g.point_of(n);
        assert!((p.x - 503).abs() <= 50);
        assert!((p.y - 397).abs() <= 50);
        assert!(g.in_bounds(n));
    }

    #[test]
    fn bounds_check() {
        let g = Grid::new(IntBox::new(0, 0, 1000, 1000), 100, 2);
        assert!(g.node_count() > 0);
        assert!(!g.in_bounds(Node { layer: 5, col: 0, row: 0 }));
    }
}

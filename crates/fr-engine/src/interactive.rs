//! Interactive (manual) routing session over the free-angle room/door model.
//!
//! Wraps the room/door single-connection router into a stateful session the GUI drives:
//! the user picks a start (a pad or trace end), then moves the cursor to a target point;
//! the session computes a clear any-angle (or snapped) route preview from the start to the
//! cursor, and commits it to the board on click. The committed trace/vias are stamped into
//! the obstacle index so subsequent manual or batch routes see them.
//!
//! This is the engine half of the GUI's manual push/shove routing and snap-angle / pad-
//! exit controls (the GUI supplies the angle restriction and start/target).

use fr_board::Board;
use fr_geometry::{IntBox, Point};
use fr_route::{
    route_connection_roomdoor, AngleRestriction, ObstacleIndex, RoomDoorOptions, RoutedConnection,
};

use crate::{build_obstacle_index, ensure_via_padstack_pub};

/// A live manual-routing session: holds the obstacle index built from the board plus the
/// routing parameters, and the current start anchor.
pub struct InteractiveRouter {
    index: ObstacleIndex,
    bound: IntBox,
    layers: usize,
    width: i64,
    clearance: i64,
    via_radius: i64,
    via_padstack: usize,
    /// The current route start (None until the user picks one).
    start: Option<(Point, usize)>,
    net: u32,
    angle: AngleRestriction,
    allow_vias: bool,
}

impl InteractiveRouter {
    /// Build a session from the current board (snapshots its copper into the index).
    pub fn new(board: &mut Board) -> InteractiveRouter {
        let layers = board.layer_count().max(1);
        let bound = board
            .outline_box()
            .or_else(|| IntBox::bound(board.pins.iter().map(|p| p.location)))
            .unwrap_or(IntBox::new(0, 0, 1, 1));
        let via_padstack = ensure_via_padstack_pub(board);
        let via_radius = fr_route::via_radius(board, via_padstack, 0).max(1);
        let index = build_obstacle_index(board, layers);
        InteractiveRouter {
            index,
            bound,
            layers,
            width: board.rules.default_width.max(1),
            clearance: board.rules.default_clearance,
            via_radius,
            via_padstack,
            start: None,
            net: 0,
            angle: AngleRestriction::None,
            allow_vias: true,
        }
    }

    pub fn set_angle(&mut self, angle: AngleRestriction) {
        self.angle = angle;
    }
    pub fn set_allow_vias(&mut self, allow: bool) {
        self.allow_vias = allow;
    }
    pub fn angle(&self) -> AngleRestriction {
        self.angle
    }
    pub fn allow_vias(&self) -> bool {
        self.allow_vias
    }
    pub fn has_start(&self) -> bool {
        self.start.is_some()
    }
    pub fn start_point(&self) -> Option<(Point, usize)> {
        self.start
    }

    /// Begin a route at `pt` on `layer` for `net`.
    pub fn begin(&mut self, pt: Point, layer: usize, net: u32) {
        self.start = Some((pt, layer));
        self.net = net;
    }

    /// Cancel the in-progress route (keeps the index as-is).
    pub fn cancel(&mut self) {
        self.start = None;
    }

    fn opts(&self) -> RoomDoorOptions {
        RoomDoorOptions {
            width: self.width,
            clearance: self.clearance,
            bound: self.bound,
            max_rooms: 4000,
            angle: self.angle,
            layers: self.layers,
            allow_vias: self.allow_vias,
            via_radius: self.via_radius,
            via_padstack: self.via_padstack,
        }
    }

    /// Compute (without committing) the route from the current start to `target` on
    /// `target_layer`. Returns the geometry for preview, or None if no clear route exists.
    pub fn preview(&self, target: Point, target_layer: usize) -> Option<RoutedConnection> {
        let (start, slayer) = self.start?;
        route_connection_roomdoor(
            &self.index, slayer, target_layer, self.net, start, target, &self.opts(),
        )
    }

    /// Commit a route from the current start to `target` on `target_layer`: append its
    /// traces/vias to the board, stamp them into the index, and move the start anchor to
    /// `target` so the user can chain segments. Returns true if a route was committed.
    pub fn commit(&mut self, board: &mut Board, target: Point, target_layer: usize) -> bool {
        let Some(conn) = self.preview(target, target_layer) else {
            return false;
        };
        for t in &conn.traces {
            self.index.add_trace(t.layer, &t.corners, t.width, self.net);
        }
        for v in &conn.vias {
            self.index
                .add_via(0, self.layers - 1, v.location, self.via_radius, self.net);
        }
        board.traces.extend(conn.traces);
        board.vias.extend(conn.vias);
        self.start = Some((target, target_layer));
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fr_board::{Component, Layer, LayerStack, Net, Resolution, Unit};

    fn board() -> Board {
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
        b.components.push(Component { name: "U1".into(), image: "I".into(), location: Point::new(1_000_000, 2_500_000), front: true, rotation: 0.0 });
        b.nets.add(Net { name: "N".into(), pins: vec![] });
        b
    }

    #[test]
    fn begin_preview_commit_appends_a_trace() {
        let mut b = board();
        let mut r = InteractiveRouter::new(&mut b);
        r.begin(Point::new(500_000, 2_500_000), 0, 0);
        assert!(r.has_start());
        let target = Point::new(4_000_000, 2_500_000);
        // preview yields a clear route in open space
        let prev = r.preview(target, 0).expect("preview route");
        assert!(!prev.traces.is_empty());
        // commit appends to the board and advances the anchor
        let traces_before = b.traces.len();
        assert!(r.commit(&mut b, target, 0));
        assert!(b.traces.len() > traces_before);
        assert_eq!(r.start_point().unwrap().0, target, "anchor advances to target");
    }

    #[test]
    fn ninety_degree_preview_is_axis_aligned() {
        let mut b = board();
        let mut r = InteractiveRouter::new(&mut b);
        r.set_angle(AngleRestriction::Ninety);
        r.begin(Point::new(500_000, 2_500_000), 0, 0);
        let prev = r.preview(Point::new(3_000_000, 3_500_000), 0).expect("route");
        for t in &prev.traces {
            for w in t.corners.windows(2) {
                let dx = (w[1].x - w[0].x).abs();
                let dy = (w[1].y - w[0].y).abs();
                assert!(dx == 0 || dy == 0, "90-deg manual route must be axis-aligned");
            }
        }
    }
}

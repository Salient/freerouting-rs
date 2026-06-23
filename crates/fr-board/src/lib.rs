//! fr-board: the board data model - layers, padstacks (incl. shapeless/no-copper),
//! components, pins, traces, vias, nets, rules, board outline. Phase 3.

mod board;
mod item;
mod layer;
mod net;
mod padstack;
mod rules;
mod units;

pub use board::{Board, Component, Keepout};
pub use item::{FixedState, Pin, Trace, Via};
pub use layer::{Direction, Layer, LayerStack};
pub use net::{Net, NetClass, NetSet};
pub use padstack::{PadShape, Padstack, PadstackSet};
pub use rules::Rules;
pub use units::{Resolution, Unit};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn build_a_small_board() {
        let mut b = Board::new("demo".into(), Resolution::new(Unit::Mil, 10000));
        b.layers = LayerStack::new(vec![
            Layer { name: "TopLayer".into(), index: 0, is_signal: true, preferred: Some(Direction::Horizontal) },
            Layer { name: "BottomLayer".into(), index: 1, is_signal: true, preferred: Some(Direction::Vertical) },
        ]);
        let pad = b.padstacks.add(Padstack::shapeless("MH1".into(), 2));
        assert!(b.padstacks.get(pad).unwrap().is_empty());
        let net = b.nets.add(Net { name: "GND".into(), pins: vec![] });
        assert_eq!(b.nets.get(net).unwrap().name, "GND");
        assert_eq!(b.layer_count(), 2);
    }
}

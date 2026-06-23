//! The top-level board model: everything read from a DSN and produced by routing.

use fr_geometry::{IntBox, Point};

use crate::item::{Pin, Trace, Via};
use crate::layer::LayerStack;
use crate::net::NetSet;
use crate::padstack::PadstackSet;
use crate::rules::Rules;
use crate::units::Resolution;

/// A placed component instance.
#[derive(Clone, Debug)]
pub struct Component {
    /// Reference designator (e.g. "R49").
    pub name: String,
    /// The library image this component instantiates (its footprint).
    pub image: String,
    pub location: Point,
    pub front: bool,
    /// Placement rotation in degrees, counter-clockwise.
    pub rotation: f64,
}

/// The board: the unit of work the router operates on.
#[derive(Clone, Debug)]
pub struct Board {
    pub name: String,
    pub resolution: Resolution,
    pub layers: LayerStack,
    pub padstacks: PadstackSet,
    pub nets: NetSet,
    pub rules: Rules,
    pub components: Vec<Component>,
    pub pins: Vec<Pin>,
    /// Board outline polygon corners (the routable boundary).
    pub outline: Vec<Point>,
    /// Routed traces (empty for an unrouted board; filled by the router).
    pub traces: Vec<Trace>,
    /// Routed vias.
    pub vias: Vec<Via>,
    /// Keepout regions: routing is forbidden inside these polygons on their layer(s).
    pub keepouts: Vec<Keepout>,
}

/// A keepout region: a polygon on a specific layer (or all layers if `layer` is None)
/// inside which routing is forbidden. Parsed from the DSN `(keepout ...)` scopes.
#[derive(Clone, Debug)]
pub struct Keepout {
    /// Layer index, or None for all layers (a `pcb`/`signal` keepout).
    pub layer: Option<usize>,
    /// Polygon corners (board units). A rectangle is stored as its 4 corners.
    pub polygon: Vec<Point>,
}

impl Board {
    pub fn new(name: String, resolution: Resolution) -> Board {
        Board {
            name,
            resolution,
            layers: LayerStack::default(),
            padstacks: PadstackSet::default(),
            nets: NetSet::default(),
            rules: Rules::default(),
            components: Vec::new(),
            pins: Vec::new(),
            outline: Vec::new(),
            traces: Vec::new(),
            vias: Vec::new(),
            keepouts: Vec::new(),
        }
    }

    /// Bounding box of the board outline, or None if no outline.
    pub fn outline_box(&self) -> Option<IntBox> {
        IntBox::bound(self.outline.iter().copied())
    }

    pub fn layer_count(&self) -> usize {
        self.layers.count()
    }

    /// All pins belonging to a given net id.
    pub fn pins_of_net(&self, net: usize) -> impl Iterator<Item = &Pin> {
        self.pins.iter().filter(move |p| p.net == Some(net))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::units::Unit;

    #[test]
    fn empty_board_basics() {
        let b = Board::new("test".into(), Resolution::new(Unit::Mil, 10000));
        assert_eq!(b.layer_count(), 0);
        assert!(b.outline_box().is_none());
        assert_eq!(b.traces.len(), 0);
    }
}

//! Layer stack.

/// A copper layer. Index 0 is the top (component side); higher indices go down to the
/// bottom layer. Routing direction preference (from Altium's LayerDirections) biases
/// the A* cost so traces prefer horizontal/vertical per layer.
#[derive(Clone, PartialEq, Eq, Debug)]
pub struct Layer {
    pub name: String,
    pub index: usize,
    pub is_signal: bool,
    pub preferred: Option<Direction>,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Direction {
    Horizontal,
    Vertical,
}

impl Direction {
    pub fn from_str(s: &str) -> Option<Direction> {
        match s.to_ascii_lowercase().as_str() {
            "horizontal" => Some(Direction::Horizontal),
            "vertical" => Some(Direction::Vertical),
            _ => None,
        }
    }
}

/// The ordered set of layers on the board.
#[derive(Clone, Debug, Default)]
pub struct LayerStack {
    layers: Vec<Layer>,
}

impl LayerStack {
    pub fn new(layers: Vec<Layer>) -> LayerStack {
        LayerStack { layers }
    }

    pub fn count(&self) -> usize {
        self.layers.len()
    }

    pub fn is_empty(&self) -> bool {
        self.layers.is_empty()
    }

    pub fn get(&self, index: usize) -> Option<&Layer> {
        self.layers.get(index)
    }

    pub fn layers(&self) -> &[Layer] {
        &self.layers
    }

    /// Layer index by name (case-sensitive, as Altium net/layer names are).
    pub fn index_of(&self, name: &str) -> Option<usize> {
        self.layers.iter().position(|l| l.name == name)
    }

    pub fn signal_layers(&self) -> impl Iterator<Item = &Layer> {
        self.layers.iter().filter(|l| l.is_signal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stack() -> LayerStack {
        LayerStack::new(vec![
            Layer { name: "TopLayer".into(), index: 0, is_signal: true, preferred: Some(Direction::Horizontal) },
            Layer { name: "BottomLayer".into(), index: 1, is_signal: true, preferred: Some(Direction::Vertical) },
        ])
    }

    #[test]
    fn lookup_by_name() {
        let s = stack();
        assert_eq!(s.index_of("TopLayer"), Some(0));
        assert_eq!(s.index_of("BottomLayer"), Some(1));
        assert_eq!(s.index_of("Nope"), None);
        assert_eq!(s.count(), 2);
        assert_eq!(s.signal_layers().count(), 2);
    }
}

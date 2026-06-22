//! Padstacks: the per-layer copper shapes of pads and vias.
//!
//! A padstack has an optional shape per layer (`None` = no copper on that layer). Some
//! Altium exports define **shapeless** padstacks (mounting holes / NPTH / fiducials)
//! with no shape on any layer; these must be representable so pin references resolve,
//! but they carry no copper and are skipped during board-item insertion
//! (ALTIUM_COMPAT.md sec 4).

use fr_geometry::ConvexTile;

/// A shape on a padstack layer. We keep the original radius for circles (vias) so the
/// writer can reproduce the padstack geometry without re-deriving it from the polygon.
#[derive(Clone, Debug)]
pub enum PadShape {
    /// A circular pad/via of the given radius (board units), centered at the origin.
    Circle { radius: i64 },
    /// A general convex copper shape (relative to the pad origin).
    Convex(ConvexTile),
}

#[derive(Clone, Debug)]
pub struct Padstack {
    pub name: String,
    /// One optional shape per board layer (index matches LayerStack). `None` = no copper.
    pub shapes: Vec<Option<PadShape>>,
    pub drillable: bool,
}

impl Padstack {
    /// A shapeless (no-copper) padstack with `layer_count` empty layers.
    pub fn shapeless(name: String, layer_count: usize) -> Padstack {
        Padstack { name, shapes: vec![None; layer_count], drillable: true }
    }

    /// True if the padstack has no shape on any layer (Altium mounting hole / NPTH /
    /// fiducial). Such pins must be skipped during board-item insertion.
    pub fn is_empty(&self) -> bool {
        self.shapes.iter().all(|s| s.is_none())
    }

    /// Lowest layer index carrying copper, or None if shapeless.
    pub fn from_layer(&self) -> Option<usize> {
        self.shapes.iter().position(|s| s.is_some())
    }

    /// Highest layer index carrying copper, or None if shapeless.
    pub fn to_layer(&self) -> Option<usize> {
        self.shapes.iter().rposition(|s| s.is_some())
    }
}

/// A library of padstacks, looked up by name (as referenced by pins and vias).
#[derive(Clone, Debug, Default)]
pub struct PadstackSet {
    stacks: Vec<Padstack>,
}

impl PadstackSet {
    pub fn add(&mut self, p: Padstack) -> usize {
        self.stacks.push(p);
        self.stacks.len() - 1
    }

    pub fn get(&self, index: usize) -> Option<&Padstack> {
        self.stacks.get(index)
    }

    pub fn index_of(&self, name: &str) -> Option<usize> {
        self.stacks.iter().position(|p| p.name == name)
    }

    pub fn by_name(&self, name: &str) -> Option<&Padstack> {
        self.stacks.iter().find(|p| p.name == name)
    }

    /// All circle-shape radii across all padstacks (for sizing the routing grid so it
    /// can resolve the largest pad's clearance).
    pub fn iter_radii(&self) -> impl Iterator<Item = i64> + '_ {
        self.stacks.iter().flat_map(|p| {
            p.shapes.iter().filter_map(|s| match s {
                Some(PadShape::Circle { radius }) => Some(*radius),
                _ => None,
            })
        })
    }

    pub fn len(&self) -> usize {
        self.stacks.len()
    }

    pub fn is_empty(&self) -> bool {
        self.stacks.is_empty()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shapeless_padstack_is_empty() {
        let p = Padstack::shapeless("Pad231".into(), 6);
        assert!(p.is_empty());
        assert_eq!(p.from_layer(), None);
        assert_eq!(p.to_layer(), None);
    }

    #[test]
    fn via_spanning_all_layers() {
        let mut shapes = vec![None; 6];
        for s in shapes.iter_mut() {
            *s = Some(PadShape::Circle { radius: 120_000 });
        }
        let p = Padstack { name: "Via1".into(), shapes, drillable: true };
        assert!(!p.is_empty());
        assert_eq!(p.from_layer(), Some(0));
        assert_eq!(p.to_layer(), Some(5));
    }

    #[test]
    fn padstack_set_lookup() {
        let mut set = PadstackSet::default();
        set.add(Padstack::shapeless("A".into(), 2));
        set.add(Padstack::shapeless("B".into(), 2));
        assert_eq!(set.index_of("B"), Some(1));
        assert!(set.by_name("A").is_some());
        assert!(set.by_name("Z").is_none());
    }
}

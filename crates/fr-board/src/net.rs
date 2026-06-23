//! Nets and net classes.

/// A net: a named set of connection points that must be electrically joined.
#[derive(Clone, Debug)]
pub struct Net {
    pub name: String,
    /// Pin references "Component-Pin" that belong to this net (as listed in the DSN).
    pub pins: Vec<String>,
}

/// A net class: a named group of nets that may share a trace width / clearance rule. From
/// the DSN `(class ...)` scope. `width`/`clearance` are in board units; `None` means "use
/// the board's global default rule".
#[derive(Clone, Debug)]
pub struct NetClass {
    pub name: String,
    /// Net names belonging to this class (as written in the DSN).
    pub nets: Vec<String>,
    pub width: Option<i64>,
    pub clearance: Option<i64>,
}

#[derive(Clone, Debug, Default)]
pub struct NetSet {
    nets: Vec<Net>,
}

impl NetSet {
    pub fn add(&mut self, n: Net) -> usize {
        self.nets.push(n);
        self.nets.len() - 1
    }

    pub fn get(&self, id: usize) -> Option<&Net> {
        self.nets.get(id)
    }

    pub fn index_of(&self, name: &str) -> Option<usize> {
        self.nets.iter().position(|n| n.name == name)
    }

    pub fn len(&self) -> usize {
        self.nets.len()
    }

    pub fn is_empty(&self) -> bool {
        self.nets.is_empty()
    }

    pub fn iter(&self) -> impl Iterator<Item = (usize, &Net)> {
        self.nets.iter().enumerate()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_and_lookup() {
        let mut s = NetSet::default();
        let id = s.add(Net { name: "GND".into(), pins: vec!["U1-1".into()] });
        assert_eq!(s.index_of("GND"), Some(id));
        assert_eq!(s.get(id).unwrap().pins.len(), 1);
    }
}

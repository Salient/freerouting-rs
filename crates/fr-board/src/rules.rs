//! Design rules: clearances and default trace widths.
//!
//! Kept simple for now - a global default clearance + width, with optional per-net-class
//! overrides. The router reads these to size traces and enforce spacing. Matches the
//! single global `(rule (width W) (clearance C))` that the sample Altium boards use.

#[derive(Clone, Debug)]
pub struct Rules {
    /// Default trace width in board units.
    pub default_width: i64,
    /// Default clearance between objects in board units.
    pub default_clearance: i64,
    /// Board-edge (outline) clearance in board units.
    pub edge_clearance: i64,
}

impl Default for Rules {
    fn default() -> Self {
        // Reasonable fallbacks; overwritten from the DSN structure (rule ...).
        Rules { default_width: 100_000, default_clearance: 80_000, edge_clearance: 80_000 }
    }
}

impl Rules {
    pub fn new(default_width: i64, default_clearance: i64) -> Rules {
        Rules { default_width, default_clearance, edge_clearance: default_clearance }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn defaults_and_new() {
        let r = Rules::new(100_000, 80_000);
        assert_eq!(r.default_width, 100_000);
        assert_eq!(r.default_clearance, 80_000);
        assert_eq!(r.edge_clearance, 80_000);
    }
}

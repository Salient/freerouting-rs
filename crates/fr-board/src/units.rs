//! Units and coordinate handling.
//!
//! Internally everything is in integer board units (the Specctra resolution unit).
//! For `(resolution mil 10000)`, 1 mil = 10000 board units, so a coordinate of
//! 12458.43 mil is stored as the integer 124584300. This matches what Altium's route
//! importer expects (scaled integers, NOT decimal mil) - see ALTIUM_COMPAT.md.

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Unit {
    Mil,
    Inch,
    Mm,
    Um,
}

impl Unit {
    /// Parse a Specctra unit token, case-insensitive. Defaults to None on unknown.
    pub fn from_str(s: &str) -> Option<Unit> {
        match s.to_ascii_lowercase().as_str() {
            "mil" => Some(Unit::Mil),
            "inch" => Some(Unit::Inch),
            "mm" => Some(Unit::Mm),
            "um" => Some(Unit::Um),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Unit::Mil => "mil",
            Unit::Inch => "inch",
            Unit::Mm => "mm",
            Unit::Um => "um",
        }
    }
}

/// The unit + resolution (board units per unit) read from `(resolution <unit> <n>)`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Resolution {
    pub unit: Unit,
    /// Board units per `unit` (e.g. 10000 for `mil 10000`). Always > 0.
    pub per_unit: i64,
}

impl Resolution {
    pub fn new(unit: Unit, per_unit: i64) -> Resolution {
        // Per ALTIUM_COMPAT.md hardening: a non-positive resolution would divide-by-zero
        // in coordinate transforms; default to the Specctra standard of 100.
        let per_unit = if per_unit > 0 { per_unit } else { 100 };
        Resolution { unit, per_unit }
    }

    /// Default Specctra resolution (mil, 100) used when a file omits/garbles it.
    pub fn default_mil() -> Resolution {
        Resolution { unit: Unit::Mil, per_unit: 100 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_units_case_insensitive() {
        assert_eq!(Unit::from_str("MIL"), Some(Unit::Mil));
        assert_eq!(Unit::from_str("mm"), Some(Unit::Mm));
        assert_eq!(Unit::from_str("furlong"), None);
    }

    #[test]
    fn resolution_guards_nonpositive() {
        assert_eq!(Resolution::new(Unit::Mil, 0).per_unit, 100);
        assert_eq!(Resolution::new(Unit::Mil, -5).per_unit, 100);
        assert_eq!(Resolution::new(Unit::Mil, 10000).per_unit, 10000);
    }
}

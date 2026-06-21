//! fr-dsn: tolerant Specctra DSN reader + SES/RTE writers.
//!
//! CRITICAL Altium output rules (freerouting-rs-spec/ALTIUM_COMPAT.md), all proven
//! against live Altium:
//!   - route files are a top-level `(routes ...)` scope,
//!   - CRLF line endings,
//!   - every wire/via on ONE line carrying its own `(net ...)` and `(type ...)`,
//!   - coordinates are scaled integers (resolution units), not decimal mil.
//!
//! Phase 4: the lexer + s-expression tree are in place (tolerant of the malformed
//! Altium output catalogued in ALTIUM_COMPAT.md). The DSN->board reader and the
//! SES/RTE writers build on these next.

pub mod lexer;
pub mod sexp;

pub use sexp::Sexp;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_small_dsn() {
        let src = "(pcb b (resolution MIL 10000) (structure (layer TopLayer) (layer BottomLayer)))";
        let s = Sexp::parse(src, '"');
        assert_eq!(s.head(), Some("pcb"));
        assert_eq!(s.child("resolution").unwrap().atom_args(), vec!["MIL", "10000"]);
        assert_eq!(s.child("structure").unwrap().children("layer").count(), 2);
    }
}

//! fr-dsn: tolerant Specctra DSN reader + SES/RTE writers.
//! CRITICAL Altium rules (see freerouting-rs-spec/ALTIUM_COMPAT.md): route files are a
//! top-level (routes ...) scope, CRLF line endings, and every wire/via on ONE line
//! carrying its own (net ...) and (type ...). Phase 4.

#[cfg(test)]
mod tests {
    #[test]
    fn crate_builds() { assert_eq!(2 + 2, 4); }
}

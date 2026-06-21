//! Integration test: parse a real Altium-exported DSN (43k lines, 6-layer, ~800
//! components) and assert the tolerant lexer/tree handle it correctly. This is the
//! ground-truth input for the round-trip harness.

use fr_dsn::lexer::detect_string_quote;
use fr_dsn::Sexp;

const REAL: &str = include_str!("fixtures/altium_board.dsn");

#[test]
fn parses_real_altium_board_without_panic() {
    let q = detect_string_quote(REAL).unwrap_or('"');
    let pcb = Sexp::parse(REAL, q);
    assert_eq!(pcb.head(), Some("pcb"), "top scope must be (pcb ...)");
}

#[test]
fn reads_resolution_and_layers() {
    let pcb = Sexp::parse(REAL, '"');
    // (resolution MIL 10000)
    let res = pcb.child("resolution").expect("resolution scope");
    let args = res.atom_args();
    assert_eq!(args.len(), 2, "resolution has unit + value");
    assert_eq!(args[0].to_ascii_uppercase(), "MIL");
    assert_eq!(args[1], "10000");

    // structure has 6 signal layers (TopLayer, MidLayer1..4, BottomLayer)
    let structure = pcb.child("structure").expect("structure scope");
    let layers: Vec<_> = structure.children("layer").collect();
    assert_eq!(layers.len(), 6, "expected 6 layers, found {}", layers.len());
}

#[test]
fn reads_network_nets() {
    let pcb = Sexp::parse(REAL, '"');
    let network = pcb.child("network").expect("network scope");
    let nets: Vec<_> = network.children("net").collect();
    assert!(nets.len() > 100, "expected many nets, found {}", nets.len());
    // a known net name should be present (Altium ~SP~ encoding preserved verbatim)
    let names: Vec<&str> = nets.iter().filter_map(|n| n.atom_args().first().copied()).collect();
    assert!(
        names.iter().any(|n| n.contains("UART~SP~TO~SP~PINE")),
        "expected the UART~SP~TO~SP~PINE net to be parsed"
    );
}

#[test]
fn reads_components_in_placement() {
    let pcb = Sexp::parse(REAL, '"');
    let placement = pcb.child("placement").expect("placement scope");
    let comps: Vec<_> = placement.children("component").collect();
    assert!(comps.len() > 100, "expected many components, found {}", comps.len());
}

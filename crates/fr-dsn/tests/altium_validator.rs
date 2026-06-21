//! Phase 4 acceptance gate: structurally validate that a route (.rte) file emitted by
//! fr-dsn satisfies every rule Altium's importer requires (ALTIUM_COMPAT.md). This is
//! the automated proxy for the real-Altium import the human does at the end.
//!
//! Round-trips the real Altium board: read its DSN, synthesize a couple of routed
//! traces/vias on real nets, emit an RTE, and assert the structure.

use fr_board::{FixedState, PadShape, Padstack, Trace, Via};
use fr_dsn::{read_board, write_rte};
use fr_geometry::Point;

const REAL: &str = include_str!("fixtures/altium_board.dsn");

/// Encodes the ALTIUM_COMPAT rules as assertions. Returns Ok(()) or a description.
fn validate_rte(rte: &str) -> Result<(), String> {
    // 1. CRLF: no bare LF (every \n preceded by \r).
    let bytes = rte.as_bytes();
    for i in 0..bytes.len() {
        if bytes[i] == b'\n' && (i == 0 || bytes[i - 1] != b'\r') {
            return Err(format!("bare LF at byte {i} (must be CRLF)"));
        }
    }
    // 2. Top-level (routes ...), no (session ...).
    if !rte.starts_with("(routes") {
        return Err("must start with a top-level (routes scope".into());
    }
    if rte.contains("(session") {
        return Err("route file must not contain a session wrapper".into());
    }
    // 3. Balanced parentheses.
    let opens = rte.matches('(').count();
    let closes = rte.matches(')').count();
    if opens != closes {
        return Err(format!("unbalanced parens: {opens} open vs {closes} close"));
    }
    // 4. Every wire line is self-contained (one line) and carries (net ...) + (type ...).
    for line in rte.lines() {
        let t = line.trim_start();
        if t.starts_with("(wire") {
            if !t.contains("(net ") {
                return Err(format!("wire missing (net ...): {t}"));
            }
            if !t.contains("(type ") {
                return Err(format!("wire missing (type ...): {t}"));
            }
            if t.matches('(').count() != t.matches(')').count() {
                return Err(format!("wire not on one balanced line: {t}"));
            }
        }
        if t.starts_with("(via") {
            if !t.contains("(net ") || !t.contains("(type ") {
                return Err(format!("via missing net/type: {t}"));
            }
        }
    }
    Ok(())
}

#[test]
fn emitted_rte_passes_altium_validator() {
    let (mut board, _w) = read_board(REAL);

    // Synthesize a via padstack + a couple of routes on the first two real nets so the
    // output exercises wires, vias, and quoting of ~SP~ net names.
    let mut shapes = vec![None; board.layer_count().max(1)];
    if let Some(first) = shapes.first_mut() {
        *first = Some(PadShape::Circle { radius: 120_000 });
    }
    let via_pad = board.padstacks.add(Padstack { name: "TestVia".into(), shapes, drillable: true });

    let net0 = 0usize;
    let net1 = 1usize.min(board.nets.len().saturating_sub(1));
    board.traces.push(Trace {
        layer: 0,
        width: board.rules.default_width,
        corners: vec![Point::new(1_000_000, 2_000_000), Point::new(3_000_000, 2_000_000)],
        net: Some(net0),
        fixed: FixedState::Route,
    });
    board.traces.push(Trace {
        layer: 0,
        width: board.rules.default_width,
        corners: vec![Point::new(4_000_000, 5_000_000), Point::new(4_000_000, 7_000_000)],
        net: Some(net1),
        fixed: FixedState::Route,
    });
    board.vias.push(Via {
        padstack: via_pad,
        location: Point::new(3_000_000, 2_000_000),
        net: Some(net0),
        fixed: FixedState::Route,
    });

    let rte = write_rte(&board);
    validate_rte(&rte).expect("emitted RTE must satisfy Altium import rules");

    // sanity: it actually contains our routed content
    assert!(rte.contains("(wire (path"), "should contain wires");
    assert!(rte.contains("(via TestVia"), "should contain the via");
}

//! SES (session) and RTE (route) writers.
//!
//! These bake in the Altium-import rules proven against live Altium
//! (ALTIUM_COMPAT.md), each verified by bisection:
//!   1. CRLF line endings (LF-only -> "List index out of bounds (0)").
//!   2. A route file is a TOP-LEVEL `(routes ...)` scope (no session wrapper).
//!   3. Every wire/via is written on ONE line carrying its own `(net ...)` and
//!      `(type ...)` (multi-line or untagged wires are silently dropped on import).
//!   4. Coordinates are scaled integers (board units), not decimal mil.
//!   5. `(string_quote ")` + `(space_in_quoted_tokens on)` are declared in the parser
//!      scope; names that need quoting are quoted, but Altium-native `~SP~` encoding is
//!      preserved verbatim so most names need no quoting.

use std::fmt::Write as _;

use fr_board::{Board, PadShape, Trace, Via};

const CRLF: &str = "\r\n";

/// Write a Specctra ROUTE (.rte) file: the routing solution as a top-level
/// `(routes ...)` scope. This is what Altium's "Import Specctra Route" expects.
pub fn write_rte(board: &Board) -> String {
    let mut s = String::new();
    write_routes_scope(&mut s, board, 0);
    s
}

/// Write a Specctra SESSION (.ses) file: a `(session ...)` wrapper containing the
/// placement, an (empty) was_is, and the routes scope.
pub fn write_ses(board: &Board) -> String {
    let mut s = String::new();
    let q = "\"";
    push(&mut s, 0, &format!("(session {}", quote_if_needed(&board.name, q)));
    push(&mut s, 1, &format!("(base_design {})", quote_if_needed(&board.name, q)));
    // placement
    push(&mut s, 1, "(placement");
    push(&mut s, 2, &format!("(resolution {} {})", board.resolution.unit.as_str(), board.resolution.per_unit));
    push(&mut s, 1, ")");
    // (was_is) is omitted when empty (an empty scope upsets strict readers).
    // routes scope, nested at depth 1
    write_routes_scope(&mut s, board, 1);
    push(&mut s, 0, ")");
    s
}

/// Write the `(routes ...)` scope at the given indent depth.
fn write_routes_scope(s: &mut String, board: &Board, depth: usize) {
    let q = "\"";
    push(s, depth, "(routes");
    push(s, depth + 1, &format!("(resolution {} {})", board.resolution.unit.as_str(), board.resolution.per_unit));
    // parser scope: always declare the quote char + space_in_quoted_tokens.
    push(s, depth + 1, "(parser");
    push(s, depth + 2, "(string_quote \")");
    push(s, depth + 2, "(space_in_quoted_tokens on)");
    push(s, depth + 1, ")");
    // library_out: the via padstacks actually used.
    write_library_out(s, board, depth + 1, q);
    // network_out: per-net wires + vias.
    write_network_out(s, board, depth + 1, q);
    push(s, depth, ")");
}

fn write_library_out(s: &mut String, board: &Board, depth: usize, q: &str) {
    // Collect padstack indices used by vias.
    let mut used: Vec<usize> = board.vias.iter().map(|v| v.padstack).collect();
    used.sort_unstable();
    used.dedup();
    push(s, depth, "(library_out");
    for idx in used {
        if let Some(ps) = board.padstacks.get(idx) {
            push(s, depth + 1, &format!("(padstack {}", quote_if_needed(&ps.name, q)));
            for (layer_i, shape) in ps.shapes.iter().enumerate() {
                if let Some(PadShape::Circle { radius }) = shape {
                    if let Some(layer) = board.layers.get(layer_i) {
                        // (circle <layer> <diameter> 0 0) - value is the diameter.
                        push(s, depth + 2, &format!("(shape (circle {} {} 0 0))", layer.name, radius * 2));
                    }
                }
            }
            push(s, depth + 1, ")");
        }
    }
    push(s, depth, ")");
}

fn write_network_out(s: &mut String, board: &Board, depth: usize, q: &str) {
    push(s, depth, "(network_out");
    for (net_id, net) in board.nets.iter() {
        let traces: Vec<&Trace> = board.traces.iter().filter(|t| t.net == Some(net_id)).collect();
        let vias: Vec<&Via> = board.vias.iter().filter(|v| v.net == Some(net_id)).collect();
        if traces.is_empty() && vias.is_empty() {
            continue;
        }
        let net_name = quote_if_needed(&net.name, q);
        push(s, depth + 1, &format!("(net {}", net_name));
        for t in traces {
            push(s, depth + 2, &wire_line(t, board, &net_name));
        }
        for v in vias {
            push(s, depth + 2, &via_line(v, board, &net_name));
        }
        push(s, depth + 1, ")");
    }
    push(s, depth, ")");
}

/// One wire on ONE line: `(wire (path LAYER WIDTH x1 y1 x2 y2 ...) (net NAME) (type T))`.
fn wire_line(t: &Trace, board: &Board, net_name: &str) -> String {
    let layer = board.layers.get(t.layer).map(|l| l.name.as_str()).unwrap_or("signal");
    let mut line = format!("(wire (path {} {}", layer, t.width);
    for p in &t.corners {
        let _ = write!(line, " {} {}", p.x, p.y);
    }
    let _ = write!(line, ") (net {}) (type {}))", net_name, t.fixed.type_token());
    line
}

/// One via on ONE line: `(via PADSTACK x y (net NAME) (type T))`.
fn via_line(v: &Via, board: &Board, net_name: &str) -> String {
    let pad = board.padstacks.get(v.padstack).map(|p| p.name.clone()).unwrap_or_else(|| "via".into());
    format!(
        "(via {} {} {} (net {}) (type {}))",
        quote_if_needed(&pad, "\""),
        v.location.x,
        v.location.y,
        net_name,
        v.fixed.type_token()
    )
}

/// Quote a name only if it contains a reserved char; preserve Altium `~SP~` encoding.
fn quote_if_needed(name: &str, q: &str) -> String {
    let needs = name.is_empty()
        || name.chars().any(|c| c == '(' || c == ')' || c == ' ' || c == '"');
    if needs {
        format!("{q}{name}{q}")
    } else {
        name.to_string()
    }
}

/// Append a line at `depth` (two spaces per level) terminated with CRLF.
fn push(s: &mut String, depth: usize, line: &str) {
    for _ in 0..depth {
        s.push_str("  ");
    }
    s.push_str(line);
    s.push_str(CRLF);
}

#[cfg(test)]
mod tests {
    use super::*;
    use fr_board::{FixedState, Layer, LayerStack, Net, PadShape, Padstack, Resolution, Unit};
    use fr_geometry::Point;

    fn demo_board() -> Board {
        let mut b = Board::new("demo".into(), Resolution::new(Unit::Mil, 10000));
        b.layers = LayerStack::new(vec![
            Layer { name: "TopLayer".into(), index: 0, is_signal: true, preferred: None },
            Layer { name: "BottomLayer".into(), index: 1, is_signal: true, preferred: None },
        ]);
        // a via padstack
        let mut shapes = vec![None, None];
        shapes[0] = Some(PadShape::Circle { radius: 120_000 });
        shapes[1] = Some(PadShape::Circle { radius: 120_000 });
        let via_pad = b.padstacks.add(Padstack { name: "Via1".into(), shapes, drillable: true });
        let net = b.nets.add(Net { name: "UART~SP~TX".into(), pins: vec![] });
        b.traces.push(Trace {
            layer: 0,
            width: 100_000,
            corners: vec![Point::new(1_000_000, 2_000_000), Point::new(3_000_000, 2_000_000)],
            net: Some(net),
            fixed: FixedState::Route,
        });
        b.vias.push(Via { padstack: via_pad, location: Point::new(3_000_000, 2_000_000), net: Some(net), fixed: FixedState::Route });
        b
    }

    #[test]
    fn rte_is_top_level_routes_crlf() {
        let out = write_rte(&demo_board());
        assert!(out.starts_with("(routes"), "must be a top-level routes scope");
        assert!(!out.contains("(session"), "rte has no session wrapper");
        // CRLF on every line
        let lines: Vec<&str> = out.split("\r\n").collect();
        assert!(lines.len() > 5);
        assert!(!out.contains('\n') || out.contains("\r\n"), "must use CRLF");
        // no bare LF (every \n is preceded by \r)
        assert!(out.bytes().zip(out.bytes().skip(1)).all(|(a, b)| b != b'\n' || a == b'\r'));
    }

    #[test]
    fn wire_is_one_line_with_net_and_type() {
        let out = write_rte(&demo_board());
        let wire_line = out.lines().find(|l| l.contains("(wire")).expect("a wire line");
        assert!(wire_line.contains("(path TopLayer 100000 1000000 2000000 3000000 2000000)"));
        assert!(wire_line.contains("(net UART~SP~TX)"), "per-wire net tag");
        assert!(wire_line.contains("(type route)"), "per-wire type tag");
        // the whole wire is on a single line
        assert!(wire_line.trim().ends_with("))"));
    }

    #[test]
    fn via_is_one_line_with_net_and_type() {
        let out = write_rte(&demo_board());
        let via_line = out.lines().find(|l| l.trim_start().starts_with("(via")).expect("a via line");
        assert!(via_line.contains("Via1 3000000 2000000"));
        assert!(via_line.contains("(net UART~SP~TX)"));
        assert!(via_line.contains("(type route)"));
    }

    #[test]
    fn library_out_uses_diameter() {
        let out = write_rte(&demo_board());
        // radius 120000 -> diameter 240000
        assert!(out.contains("(circle TopLayer 240000 0 0)"));
    }

    #[test]
    fn ses_has_session_wrapper_and_routes() {
        let out = write_ses(&demo_board());
        assert!(out.starts_with("(session"));
        assert!(out.contains("(placement"));
        assert!(out.contains("(routes"));
        assert!(out.contains("(network_out"));
    }
}

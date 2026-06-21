//! DSN -> Board reader. Walks the tolerant Sexp tree and builds an `fr_board::Board`.
//!
//! Tolerant by design (ALTIUM_COMPAT.md sec 4): shapeless padstacks are kept (no
//! copper), unknown units default to mil, non-positive resolution defaults to 100,
//! and malformed sub-scopes are skipped rather than aborting the whole parse. The
//! reader never panics on real Altium output.

use fr_board::{
    Board, Component, Direction, Layer, LayerStack, Net, PadShape, Padstack, Resolution, Rules, Unit,
};
use fr_geometry::Point;

use crate::lexer::detect_string_quote;
use crate::sexp::Sexp;

/// Parse DSN source text into a Board. Returns the board plus any non-fatal warnings.
pub fn read_board(src: &str) -> (Board, Vec<String>) {
    let quote = detect_string_quote(src).unwrap_or('"');
    let pcb = Sexp::parse(src, quote);
    let mut warnings = Vec::new();

    let name = pcb.atom_args().first().copied().unwrap_or("board").to_string();
    let resolution = read_resolution(&pcb, &mut warnings);
    let mut board = Board::new(name, resolution);

    if let Some(structure) = pcb.child("structure") {
        board.layers = read_layers(structure);
        board.rules = read_rules(structure, &board.rules, resolution, &mut warnings);
        board.outline = read_outline(structure, resolution);
    }
    let layer_count = board.layers.count().max(1);

    if let Some(library) = pcb.child("library") {
        let layers = board.layers.clone();
        read_padstacks(library, &layers, resolution, &mut board.padstacks, &mut warnings);
    }

    if let Some(placement) = pcb.child("placement") {
        read_components(placement, resolution, &mut board);
    }

    if let Some(network) = pcb.child("network") {
        read_nets(network, &mut board);
    }

    let _ = layer_count;
    (board, warnings)
}

fn read_resolution(pcb: &Sexp, warnings: &mut Vec<String>) -> Resolution {
    if let Some(res) = pcb.child("resolution") {
        let args = res.atom_args();
        let unit = args
            .first()
            .and_then(|u| Unit::from_str(u))
            .unwrap_or_else(|| {
                warnings.push("resolution: unrecognised unit; defaulting to mil".into());
                Unit::Mil
            });
        let per = args.get(1).and_then(|v| parse_int(v)).unwrap_or(100);
        return Resolution::new(unit, per);
    }
    warnings.push("no resolution scope; defaulting to mil 100".into());
    Resolution::default_mil()
}

fn read_layers(structure: &Sexp) -> LayerStack {
    let mut layers = Vec::new();
    for (i, l) in structure.children("layer").enumerate() {
        let name = l.atom_args().first().copied().unwrap_or("layer").to_string();
        let is_signal = l
            .child("type")
            .and_then(|t| t.atom_args().first().copied())
            .map(|t| t.eq_ignore_ascii_case("signal"))
            .unwrap_or(true);
        let preferred = l
            .child("direction")
            .and_then(|d| d.atom_args().first().copied())
            .and_then(Direction::from_str);
        layers.push(Layer { name, index: i, is_signal, preferred });
    }
    LayerStack::new(layers)
}

fn read_rules(structure: &Sexp, base: &Rules, res: Resolution, _warnings: &mut [String]) -> Rules {
    let mut rules = base.clone();
    if let Some(rule) = structure.child("rule") {
        // width/clearance are in design units (e.g. mil); scale to board units.
        if let Some(w) = rule.child("width").and_then(|w| w.atom_args().first().and_then(|v| parse_num(v))) {
            rules.default_width = scale(w, res);
        }
        if let Some(c) = rule.child("clearance").and_then(|c| c.atom_args().first().and_then(|v| parse_num(v))) {
            rules.default_clearance = scale(c, res);
            rules.edge_clearance = scale(c, res);
        }
    }
    rules
}

fn read_outline(structure: &Sexp, res: Resolution) -> Vec<Point> {
    // Boundary may be a (rect pcb ...) or (path pcb/signal w x y ...). Take the first
    // boundary with usable coordinates.
    for boundary in structure.children("boundary") {
        if let Some(rect) = boundary.child("rect") {
            let nums: Vec<f64> = rect.atom_args().iter().skip(1).filter_map(|s| parse_num(s)).collect();
            if nums.len() >= 4 {
                let (x1, y1, x2, y2) = (nums[0], nums[1], nums[2], nums[3]);
                return vec![
                    scale_pt(x1, y1, res),
                    scale_pt(x2, y1, res),
                    scale_pt(x2, y2, res),
                    scale_pt(x1, y2, res),
                ];
            }
        }
        if let Some(path) = boundary.child("path") {
            // path <layer> <width> x y x y ...
            let nums: Vec<f64> = path.atom_args().iter().skip(2).filter_map(|s| parse_num(s)).collect();
            if nums.len() >= 6 {
                let mut pts = Vec::new();
                let mut i = 0;
                while i + 1 < nums.len() {
                    pts.push(scale_pt(nums[i], nums[i + 1], res));
                    i += 2;
                }
                return pts;
            }
        }
    }
    Vec::new()
}

fn read_padstacks(
    library: &Sexp,
    layers: &LayerStack,
    res: Resolution,
    padstacks: &mut fr_board::PadstackSet,
    warnings: &mut Vec<String>,
) {
    let layer_count = layers.count().max(1);
    for ps in library.children("padstack") {
        let name = ps.atom_args().first().copied().unwrap_or("pad").to_string();
        let shapes_in: Vec<&Sexp> = ps.children("shape").collect();
        if shapes_in.is_empty() {
            // Shapeless padstack (Altium mounting hole / NPTH / fiducial). Keep it so
            // pin references resolve; it carries no copper.
            warnings.push(format!("padstack '{name}' has no shape; registered as no-copper"));
            padstacks.add(Padstack::shapeless(name, layer_count));
            continue;
        }
        let mut shapes: Vec<Option<PadShape>> = vec![None; layer_count];
        for shape in shapes_in {
            // shape contains one of (circle <layer> <dia> [x y]) / (rect <layer> ...) / (polygon ...)
            if let Some(circle) = shape.child("circle") {
                let a = circle.atom_args();
                let layer_tok = a.first().copied().unwrap_or("signal");
                let dia = a.get(1).and_then(|v| parse_num(v)).unwrap_or(0.0);
                let radius = (scale(dia, res) / 2).max(1);
                apply_shape(&mut shapes, layers, layer_tok, PadShape::Circle { radius });
            } else if let Some(rect) = shape.child("rect") {
                let a = rect.atom_args();
                let layer_tok = a.first().copied().unwrap_or("signal");
                // Represent rect pads as a circle of equivalent half-extent for now
                // (the router only needs an approximate keepout footprint at the pin).
                let nums: Vec<f64> = a.iter().skip(1).filter_map(|s| parse_num(s)).collect();
                if nums.len() >= 4 {
                    let hw = ((nums[2] - nums[0]).abs() / 2.0).max((nums[3] - nums[1]).abs() / 2.0);
                    let radius = scale(hw, res).max(1);
                    apply_shape(&mut shapes, layers, layer_tok, PadShape::Circle { radius });
                }
            }
            // polygon shapes: skipped for the approximate pad footprint (TODO Phase 5+)
        }
        // A shape scope that produced no copper (e.g. empty `(shape)`) is still a
        // shapeless padstack as far as the router is concerned.
        if shapes.iter().all(|s| s.is_none()) {
            warnings.push(format!("padstack '{name}' resolved to no copper; treating as no-copper"));
        }
        padstacks.add(Padstack { name, shapes, drillable: true });
    }
}

fn apply_shape(shapes: &mut [Option<PadShape>], layers: &LayerStack, layer_tok: &str, shape: PadShape) {
    if layer_tok.eq_ignore_ascii_case("signal") || layer_tok.eq_ignore_ascii_case("pcb") {
        // applies to all (signal) layers
        for (i, slot) in shapes.iter_mut().enumerate() {
            if layers.get(i).map(|l| l.is_signal).unwrap_or(true) {
                *slot = Some(shape.clone());
            }
        }
    } else if let Some(idx) = layers.index_of(layer_tok) {
        if idx < shapes.len() {
            shapes[idx] = Some(shape);
        }
    }
}

fn read_components(placement: &Sexp, res: Resolution, board: &mut Board) {
    for comp in placement.children("component") {
        for place in comp.children("place") {
            let a = place.atom_args();
            if a.len() >= 3 {
                let name = a[0].to_string();
                let x = parse_num(a[1]).unwrap_or(0.0);
                let y = parse_num(a[2]).unwrap_or(0.0);
                let front = a.get(3).map(|s| s.eq_ignore_ascii_case("front")).unwrap_or(true);
                let rotation = a.get(4).and_then(|s| parse_num(s)).unwrap_or(0.0);
                board.components.push(Component { name, location: scale_pt(x, y, res), front, rotation });
            }
        }
    }
}

fn read_nets(network: &Sexp, board: &mut Board) {
    for net in network.children("net") {
        let name = net.atom_args().first().copied().unwrap_or("net").to_string();
        let mut pins = Vec::new();
        if let Some(pins_scope) = net.child("pins") {
            // (pins  A-1 B-2 ...) - pin refs are bare atoms after the head
            for p in pins_scope.atom_args() {
                pins.push(p.to_string());
            }
        }
        board.nets.add(Net { name, pins });
    }
}

// --- numeric helpers ---

fn parse_int(s: &str) -> Option<i64> {
    s.trim().parse::<i64>().ok().or_else(|| s.trim().parse::<f64>().ok().map(|f| f as i64))
}

fn parse_num(s: &str) -> Option<f64> {
    s.trim().parse::<f64>().ok()
}

/// Scale a value in design units to integer board units.
fn scale(v: f64, res: Resolution) -> i64 {
    (v * res.per_unit as f64).round() as i64
}

fn scale_pt(x: f64, y: f64, res: Resolution) -> Point {
    Point::new(scale(x, res), scale(y, res))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_minimal_board() {
        let src = "(pcb demo (resolution mil 10000) \
            (structure (layer TopLayer (type signal)(direction horizontal)) \
                       (layer BottomLayer (type signal)(direction vertical)) \
                       (rule (width 10.0)(clearance 8.0)) \
                       (boundary (rect pcb 0 0 100 200))) \
            (library (padstack Via1 (shape (circle signal 24.0))) (padstack MH (shape))) \
            (placement (component RES (place R1 50.0 60.0 front 90))) \
            (network (net GND (pins R1-1 R1-2))))";
        let (b, _w) = read_board(src);
        assert_eq!(b.layer_count(), 2);
        assert_eq!(b.rules.default_width, 100_000); // 10.0 mil * 10000
        assert_eq!(b.rules.default_clearance, 80_000);
        // outline rect 0,0..100,200 mil -> board units
        let bb = b.outline_box().unwrap();
        assert_eq!(bb.ur.x, 1_000_000);
        assert_eq!(bb.ur.y, 2_000_000);
        // padstacks: Via1 (circle) + MH (shapeless)
        assert_eq!(b.padstacks.len(), 2);
        assert!(b.padstacks.by_name("MH").unwrap().is_empty());
        assert!(!b.padstacks.by_name("Via1").unwrap().is_empty());
        // component placed and scaled
        assert_eq!(b.components.len(), 1);
        assert_eq!(b.components[0].location, Point::new(500_000, 600_000));
        // net
        assert_eq!(b.nets.len(), 1);
        assert_eq!(b.nets.get(0).unwrap().pins, vec!["R1-1", "R1-2"]);
    }
}

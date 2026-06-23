//! DSN -> Board reader. Walks the tolerant Sexp tree and builds an `fr_board::Board`.
//!
//! Tolerant by design (ALTIUM_COMPAT.md sec 4): shapeless padstacks are kept (no
//! copper), unknown units default to mil, non-positive resolution defaults to 100,
//! and malformed sub-scopes are skipped rather than aborting the whole parse. The
//! reader never panics on real Altium output.

use fr_board::{
    Board, Component, Direction, Layer, LayerStack, Net, PadShape, Padstack, Pin, Resolution, Rules,
    Unit,
};
use fr_geometry::{ConvexTile, Point};

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

    // Parse library images (footprints + pin offsets) so we can place pins exactly.
    let mut images: std::collections::HashMap<String, Vec<ImagePin>> = std::collections::HashMap::new();
    if let Some(library) = pcb.child("library") {
        let layers = board.layers.clone();
        read_padstacks(library, &layers, resolution, &mut board.padstacks, &mut warnings);
        images = read_images(library, resolution);
    }

    if let Some(placement) = pcb.child("placement") {
        read_components(placement, resolution, &mut board);
    }

    if let Some(network) = pcb.child("network") {
        read_nets(network, &mut board);
    }

    // Build real pins: for each placed component, transform its image's pin offsets by
    // the component placement (location + rotation + front/back mirror). This replaces
    // the old "all pins at component center" approximation and is what lets the router
    // see distinct, correctly-located endpoints.
    build_pins(&mut board, &images, &mut warnings);

    let _ = layer_count;
    (board, warnings)
}

/// A pin within a library image: padstack name, pin name, offset from the image origin.
struct ImagePin {
    padstack: String,
    name: String,
    offset: Point,
}

fn read_images(library: &Sexp, res: Resolution) -> std::collections::HashMap<String, Vec<ImagePin>> {
    let mut map = std::collections::HashMap::new();
    for image in library.children("image") {
        let img_name = image.atom_args().first().copied().unwrap_or("").to_string();
        let mut pins = Vec::new();
        for pin in image.children("pin") {
            // (pin <padstack> <pinname> dx dy [(rotate r)])
            let a = pin.atom_args();
            if a.len() >= 4 {
                let padstack = a[0].to_string();
                let name = a[1].to_string();
                let dx = parse_num(a[2]).unwrap_or(0.0);
                let dy = parse_num(a[3]).unwrap_or(0.0);
                pins.push(ImagePin { padstack, name, offset: scale_pt(dx, dy, res) });
            }
        }
        map.insert(img_name, pins);
    }
    map
}

/// Rotate a point (about origin) by `deg` degrees CCW, then mirror X for back side.
fn transform_offset(off: Point, deg: f64, front: bool) -> Point {
    let r = deg.to_radians();
    let (s, c) = r.sin_cos();
    let (ox, oy) = (off.x as f64, off.y as f64);
    let mut x = ox * c - oy * s;
    let y = ox * s + oy * c;
    if !front {
        x = -x; // back-side components are mirrored about the Y axis
    }
    Point::new(x.round() as i64, y.round() as i64)
}

fn build_pins(
    board: &mut Board,
    images: &std::collections::HashMap<String, Vec<ImagePin>>,
    warnings: &mut Vec<String>,
) {
    // Map "RefDes-PinName" -> net id from the netlist.
    let mut pin_net: std::collections::HashMap<String, usize> = std::collections::HashMap::new();
    for net_id in 0..board.nets.len() {
        for pref in &board.nets.get(net_id).unwrap().pins {
            pin_net.insert(pref.clone(), net_id);
        }
    }

    let mut pins = Vec::new();
    let mut missing_images = 0usize;
    // collect component data first (avoid borrow conflict while pushing pins)
    let comps: Vec<(String, String, Point, bool, f64)> = board
        .components
        .iter()
        .map(|c| (c.name.clone(), c.image.clone(), c.location, c.front, c.rotation))
        .collect();
    for (refdes, image, loc, front, rot) in comps {
        let Some(img_pins) = images.get(&image) else {
            missing_images += 1;
            continue;
        };
        for ip in img_pins {
            let t = transform_offset(ip.offset, rot, front);
            let world = Point::new(loc.x + t.x, loc.y + t.y);
            let pref = format!("{refdes}-{}", ip.name);
            let net = pin_net.get(&pref).copied();
            let padstack = board.padstacks.index_of(&ip.padstack).unwrap_or(usize::MAX);
            pins.push(Pin {
                component: refdes.clone(),
                name: ip.name.clone(),
                padstack,
                location: world,
                net,
                rotation: rot,
                front,
            });
        }
    }
    if missing_images > 0 {
        warnings.push(format!("{missing_images} components had no matching library image"));
    }
    board.pins = pins;
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
    // A board may have several (boundary ...) scopes. A `(path ...)` boundary traces the
    // TRUE board edge (often a concave polygon, e.g. an L-shaped board), while a
    // `(rect pcb ...)` boundary is usually just the rectangular sheet/extent that bounds
    // it. Prefer the most detailed path boundary; fall back to a rect only if no usable
    // path exists. (Altium exports both; taking the rect drew the wrong, rectangular
    // shape.)
    let mut best_path: Vec<Point> = Vec::new();
    let mut rect_fallback: Vec<Point> = Vec::new();
    for boundary in structure.children("boundary") {
        if let Some(path) = boundary.child("path") {
            // path <layer> <width> x y x y ...
            let nums: Vec<f64> = path.atom_args().iter().skip(2).filter_map(|s| parse_num(s)).collect();
            if nums.len() >= 6 {
                let mut pts: Vec<Point> = nums
                    .chunks_exact(2)
                    .map(|c| scale_pt(c[0], c[1], res))
                    .collect();
                // a closing repeat of the first vertex is common; drop it.
                if pts.len() >= 2 && pts.first() == pts.last() {
                    pts.pop();
                }
                // keep the path with the most vertices (the most detailed outline).
                if pts.len() > best_path.len() {
                    best_path = pts;
                }
            }
        } else if let Some(rect) = boundary.child("rect") {
            let nums: Vec<f64> = rect.atom_args().iter().skip(1).filter_map(|s| parse_num(s)).collect();
            if nums.len() >= 4 && rect_fallback.is_empty() {
                let (x1, y1, x2, y2) = (nums[0], nums[1], nums[2], nums[3]);
                rect_fallback = vec![
                    scale_pt(x1, y1, res),
                    scale_pt(x2, y1, res),
                    scale_pt(x2, y2, res),
                    scale_pt(x1, y2, res),
                ];
            }
        }
    }
    if !best_path.is_empty() {
        best_path
    } else {
        rect_fallback
    }
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
                // (rect <layer> x1 y1 x2 y2): the true rectangular copper, relative to the
                // pad origin, as a CCW convex tile. Preserves real pad shape for the DRC,
                // the room/door obstacle model, and the GUI (no longer circle-approximated).
                let nums: Vec<f64> = a.iter().skip(1).filter_map(|s| parse_num(s)).collect();
                if nums.len() >= 4 {
                    let (x1, y1) = (scale(nums[0], res), scale(nums[1], res));
                    let (x2, y2) = (scale(nums[2], res), scale(nums[3], res));
                    let (lo_x, hi_x) = (x1.min(x2), x1.max(x2));
                    let (lo_y, hi_y) = (y1.min(y2), y1.max(y2));
                    let tile = ConvexTile::from_ccw(vec![
                        Point::new(lo_x, lo_y),
                        Point::new(hi_x, lo_y),
                        Point::new(hi_x, hi_y),
                        Point::new(lo_x, hi_y),
                    ]);
                    apply_shape(&mut shapes, layers, layer_tok, PadShape::Convex(tile));
                }
            } else if let Some(poly) = shape.child("polygon") {
                // (polygon <layer> <aperture_width> x1 y1 x2 y2 ...): the polygon copper
                // outline relative to the pad origin. Keep the real shape as a convex tile
                // (pad polygons are convex); ensure CCW so the model's predicates hold.
                let a = poly.atom_args();
                let layer_tok = a.first().copied().unwrap_or("signal");
                // skip the layer token + aperture width, then read coordinate pairs.
                let nums: Vec<f64> = a.iter().skip(2).filter_map(|s| parse_num(s)).collect();
                let mut verts: Vec<Point> = nums
                    .chunks_exact(2)
                    .map(|c| scale_pt(c[0], c[1], res))
                    .collect();
                // a closing repeat of the first vertex is common; drop it.
                if verts.len() >= 2 && verts.first() == verts.last() {
                    verts.pop();
                }
                if verts.len() >= 3 {
                    ensure_ccw(&mut verts);
                    apply_shape(&mut shapes, layers, layer_tok, PadShape::Convex(ConvexTile::from_ccw(verts)));
                }
            }
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
        // the component scope head is the library image name
        let image = comp.atom_args().first().copied().unwrap_or("").to_string();
        for place in comp.children("place") {
            let a = place.atom_args();
            if a.len() >= 3 {
                let name = a[0].to_string();
                let x = parse_num(a[1]).unwrap_or(0.0);
                let y = parse_num(a[2]).unwrap_or(0.0);
                let front = a.get(3).map(|s| s.eq_ignore_ascii_case("front")).unwrap_or(true);
                let rotation = a.get(4).and_then(|s| parse_num(s)).unwrap_or(0.0);
                board.components.push(Component {
                    name,
                    image: image.clone(),
                    location: scale_pt(x, y, res),
                    front,
                    rotation,
                });
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

/// Reverse `verts` in place if they are clockwise, so the polygon is CCW (the winding the
/// ConvexTile predicates and the GUI fill assume). Uses the exact integer signed area.
fn ensure_ccw(verts: &mut [Point]) {
    let n = verts.len();
    let mut area2: i128 = 0;
    for i in 0..n {
        let a = verts[i];
        let b = verts[(i + 1) % n];
        area2 += (a.x as i128) * (b.y as i128) - (b.x as i128) * (a.y as i128);
    }
    if area2 < 0 {
        verts.reverse();
    }
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

    #[test]
    fn transform_offset_rotation_and_mirror() {
        // 90 deg CCW maps (+x,0) -> (0,+x)
        let p = transform_offset(Point::new(100, 0), 90.0, true);
        assert!((p.x).abs() < 2 && (p.y - 100).abs() < 2, "got {:?}", p);
        // back-side mirrors X: (+x,0) at 0 deg -> (-x,0)
        let m = transform_offset(Point::new(100, 0), 0.0, false);
        assert_eq!(m, Point::new(-100, 0));
    }

    #[test]
    fn builds_pins_at_transformed_locations() {
        // Two-pin image, component placed at (1000000,1000000) mil-units, rotation 0,
        // front. Pins at +/-18.2087 mil on x => +/-182087 board units from center.
        let src = "(pcb demo (resolution mil 10000) \
            (structure (layer Top (type signal)) (rule (width 10)(clearance 8)) \
                       (boundary (rect pcb 0 0 1000 1000))) \
            (library (padstack P (shape (circle signal 20.0))) \
                     (image RES2 (pin P 1 -18.2087 0.0) (pin P 2 18.2087 0.0))) \
            (placement (component RES2 (place R1 100.0 100.0 front 0))) \
            (network (net N (pins R1-1 R1-2))))";
        let (b, _w) = read_board(src);
        // two pins built, at the transformed offsets around (1_000_000, 1_000_000)
        assert_eq!(b.pins.len(), 2);
        let xs: Vec<i64> = b.pins.iter().map(|p| p.location.x).collect();
        assert!(xs.contains(&(1_000_000 - 182_087)));
        assert!(xs.contains(&(1_000_000 + 182_087)));
        // both pins bound to net 0
        assert!(b.pins.iter().all(|p| p.net == Some(0)));
    }
}

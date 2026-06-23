//! DSN -> Board reader. Walks the tolerant Sexp tree and builds an `fr_board::Board`.
//!
//! Tolerant by design (ALTIUM_COMPAT.md sec 4): shapeless padstacks are kept (no
//! copper), unknown units default to mil, non-positive resolution defaults to 100,
//! and malformed sub-scopes are skipped rather than aborting the whole parse. The
//! reader never panics on real Altium output.

use fr_board::{
    Board, Component, Direction, FixedState, Layer, LayerStack, Net, NetSet, PadShape, Padstack,
    PadstackSet, Pin, Resolution, Rules, Trace, Unit, Via,
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
        board.keepouts = read_keepouts(structure, &board.layers, resolution);
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
        board.net_classes = read_classes(network, resolution);
    }

    // Build real pins: for each placed component, transform its image's pin offsets by
    // the component placement (location + rotation + front/back mirror). This replaces
    // the old "all pins at component center" approximation and is what lets the router
    // see distinct, correctly-located endpoints.
    build_pins(&mut board, &images, &mut warnings);

    // Pre-existing wiring: traces/vias already routed in the source design. Keep them as
    // fixed copper (the autorouter routes around them and won't rip them up).
    if let Some(wiring) = pcb.child("wiring") {
        read_wiring(wiring, &board.layers, &board.padstacks, &board.nets, resolution, &mut board.traces, &mut board.vias, &mut warnings);
    }

    // Import-correctness check: flag overlapping pads (a common Altium-vs-Specctra issue).
    check_pad_overlaps(&board, &mut warnings);

    let _ = layer_count;
    (board, warnings)
}

/// A pin within a library image: padstack name, pin name, offset from the image origin,
/// and the pin's own pad rotation (degrees CCW) from a `(rotate r)` token. The pad SHAPE
/// is rotated by this per-pin angle (plus the component placement rotation).
struct ImagePin {
    padstack: String,
    name: String,
    offset: Point,
    rotate: f64,
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
                // per-pin (rotate r) sub-scope, if present, rotates the pad shape.
                let rotate = pin
                    .child("rotate")
                    .and_then(|r| r.atom_args().first().and_then(|s| parse_num(s)))
                    .unwrap_or(0.0);
                pins.push(ImagePin { padstack, name, offset: scale_pt(dx, dy, res), rotate });
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
            // total pad-shape rotation = component placement rotation + per-pin rotate.
            pins.push(Pin {
                component: refdes.clone(),
                name: ip.name.clone(),
                padstack,
                location: world,
                net,
                rotation: rot + ip.rotate,
                front,
            });
        }
    }
    if missing_images > 0 {
        warnings.push(format!("{missing_images} components had no matching library image"));
    }
    board.pins = pins;
}

/// Parse the `(wiring ...)` section: pre-routed wires and vias from the source design.
/// Each `(wire (path L w x y ...) (net N) (type T))` becomes a fixed Trace; each
/// `(via padstack x y ...)` becomes a fixed Via. Nets are resolved by name; unknown
/// nets/layers are skipped with a count warning. These are kept as fixed copper so the
/// autorouter routes around them and does not rip them up.
#[allow(clippy::too_many_arguments)]
fn read_wiring(
    wiring: &Sexp,
    layers: &LayerStack,
    padstacks: &PadstackSet,
    nets: &NetSet,
    res: Resolution,
    traces: &mut Vec<Trace>,
    vias: &mut Vec<Via>,
    warnings: &mut Vec<String>,
) {
    let mut skipped = 0usize;
    for wire in wiring.children("wire") {
        let Some(path) = wire.child("path") else { continue };
        let a = path.atom_args();
        let layer_tok = a.first().copied().unwrap_or("");
        let Some(layer) = layers.index_of(layer_tok) else { skipped += 1; continue };
        // path: <layer> <width> x y x y ...
        let width = a.get(1).and_then(|s| parse_num(s)).map(|w| scale(w, res)).unwrap_or(0).max(1);
        let nums: Vec<f64> = a.iter().skip(2).filter_map(|s| parse_num(s)).collect();
        let corners: Vec<Point> = nums.chunks_exact(2).map(|c| scale_pt(c[0], c[1], res)).collect();
        if corners.len() < 2 {
            continue;
        }
        let net = wire
            .child("net")
            .and_then(|n| n.atom_args().first().copied())
            .and_then(|name| nets.index_of(name));
        traces.push(Trace { layer, width, corners, net, fixed: FixedState::Fix });
    }
    for via in wiring.children("via") {
        let a = via.atom_args();
        // (via <padstack> x y [net ...])
        if a.len() < 3 {
            continue;
        }
        let padstack = padstacks.index_of(a[0]).unwrap_or(usize::MAX);
        let (Some(x), Some(y)) = (parse_num(a[1]), parse_num(a[2])) else { continue };
        let net = via
            .child("net")
            .and_then(|n| n.atom_args().first().copied())
            .and_then(|name| nets.index_of(name));
        vias.push(Via { padstack, location: scale_pt(x, y, res), net, fixed: FixedState::Fix });
    }
    if skipped > 0 {
        warnings.push(format!("{skipped} pre-existing wire(s) on an unknown layer were skipped"));
    }
    if !traces.is_empty() || !vias.is_empty() {
        warnings.push(format!(
            "loaded {} pre-existing wire(s) + {} via(s) from the source design (kept as fixed copper)",
            traces.len(), vias.len()
        ));
    }
}

/// Import-correctness pass: detect overlapping pad geometry, a frequent Altium-export
/// issue (Altium permits some overlaps the Specctra standard does not).
///
/// Policy (per the project's import guidance):
///  - Overlapping pads on the same component with the SAME pin name are legitimately tied
///    internally (e.g. a MOSFET's multiple source pins / multiple drain pins). These are
///    expected; we note them at low volume but don't treat them as errors.
///  - Overlapping pads with DIFFERENT names (especially on a 2-pin part like a capacitor)
///    indicate a likely error — often a pad rotated so two terminals touch. These are
///    flagged so the user can fix them before export.
/// Uses each pad's circumscribed radius (per-pin location) as a cheap overlap proxy.
fn check_pad_overlaps(board: &Board, warnings: &mut Vec<String>) {
    // INSCRIBED radius of a pin's pad (largest circle that fits inside), so adjacent
    // fine-pitch IC pads — which are close but don't overlap — don't false-positive. For a
    // rectangle this is half the SHORTER side; for a circle, its radius. 0 if shapeless.
    let pad_r = |pin: &Pin| -> f64 {
        let Some(ps) = board.padstacks.get(pin.padstack) else { return 0.0 };
        ps.shapes
            .iter()
            .filter_map(|s| match s {
                Some(PadShape::Circle { radius }) => Some(*radius as f64),
                Some(PadShape::Convex(t)) => {
                    let b = t.bounding_box()?;
                    Some((b.width().min(b.height()) as f64) / 2.0)
                }
                None => None,
            })
            .fold(0.0_f64, f64::max)
    };
    // group pin indices by component.
    let mut by_comp: std::collections::HashMap<&str, Vec<usize>> = std::collections::HashMap::new();
    for (i, pin) in board.pins.iter().enumerate() {
        by_comp.entry(pin.component.as_str()).or_default().push(i);
    }
    let mut tied = 0usize;
    let mut errors = 0usize;
    let mut error_examples: Vec<String> = Vec::new();
    for (comp, idxs) in &by_comp {
        for a in 0..idxs.len() {
            for b in (a + 1)..idxs.len() {
                let (pa, pb) = (&board.pins[idxs[a]], &board.pins[idxs[b]]);
                let ra = pad_r(pa);
                let rb = pad_r(pb);
                if ra <= 0.0 || rb <= 0.0 {
                    continue;
                }
                let d = (((pa.location.x - pb.location.x) as f64).powi(2)
                    + ((pa.location.y - pb.location.y) as f64).powi(2))
                .sqrt();
                // overlap when centers are closer than the sum of radii (with slack).
                if d < (ra + rb) * 0.95 {
                    if pa.name == pb.name {
                        tied += 1; // same-net tied pads (MOSFET S/S, D/D) — expected
                    } else {
                        errors += 1;
                        if error_examples.len() < 8 {
                            error_examples.push(format!("{comp} pins {}/{}", pa.name, pb.name));
                        }
                    }
                }
            }
        }
    }
    if tied > 0 {
        warnings.push(format!(
            "{tied} overlapping same-name pad pair(s) — treated as internally-tied (e.g. MOSFET source/drain); OK"
        ));
    }
    if errors > 0 {
        warnings.push(format!(
            "{errors} overlapping DIFFERENT-name pad pair(s) — likely a pad-rotation/placement error (e.g. a 2-terminal part whose pads touch). Examples: {}",
            error_examples.join(", ")
        ));
    }
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

/// Parse `(keepout ...)` regions from the structure scope. Each keepout has a `(rect L ..)`
/// or `(path L w x y ...)` on layer `L` (or `pcb`/`signal` for all layers). Routing is
/// forbidden inside these. Both the structure-level and any nested keepouts are collected.
fn read_keepouts(structure: &Sexp, layers: &LayerStack, res: Resolution) -> Vec<fr_board::Keepout> {
    let mut out = Vec::new();
    for ko in structure.children("keepout") {
        // resolve the layer token from the rect/path child.
        let (layer_tok, polygon) = if let Some(rect) = ko.child("rect") {
            let a = rect.atom_args();
            let lt = a.first().copied().unwrap_or("signal").to_string();
            let nums: Vec<f64> = a.iter().skip(1).filter_map(|s| parse_num(s)).collect();
            if nums.len() < 4 {
                continue;
            }
            let (x1, y1, x2, y2) = (nums[0], nums[1], nums[2], nums[3]);
            (lt, vec![
                scale_pt(x1, y1, res), scale_pt(x2, y1, res),
                scale_pt(x2, y2, res), scale_pt(x1, y2, res),
            ])
        } else if let Some(path) = ko.child("path") {
            let a = path.atom_args();
            let lt = a.first().copied().unwrap_or("signal").to_string();
            // path <layer> <width> x y x y ...
            let nums: Vec<f64> = a.iter().skip(2).filter_map(|s| parse_num(s)).collect();
            let mut poly: Vec<Point> = nums.chunks_exact(2).map(|c| scale_pt(c[0], c[1], res)).collect();
            if poly.len() >= 2 && poly.first() == poly.last() {
                poly.pop();
            }
            if poly.len() < 2 {
                continue;
            }
            (lt, poly)
        } else {
            continue;
        };
        let layer = if layer_tok.eq_ignore_ascii_case("signal") || layer_tok.eq_ignore_ascii_case("pcb") {
            None
        } else {
            layers.index_of(&layer_tok)
        };
        out.push(fr_board::Keepout { layer, polygon });
    }
    out
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

/// Parse `(class NAME net1 net2 ... [(rule (width W) (clearance C))])` net-class scopes.
/// The member net names are the class scope's bare atom args (after the class name); an
/// optional `(rule ...)` child sets a per-class width/clearance (board units).
fn read_classes(network: &Sexp, res: Resolution) -> Vec<fr_board::NetClass> {
    let mut out = Vec::new();
    for class in network.children("class") {
        let args = class.atom_args();
        if args.is_empty() {
            continue;
        }
        let name = args[0].to_string();
        // remaining bare atoms are member net names.
        let nets: Vec<String> = args.iter().skip(1).map(|s| s.to_string()).collect();
        // optional per-class rule.
        let (mut width, mut clearance) = (None, None);
        if let Some(rule) = class.child("rule") {
            if let Some(w) = rule.child("width") {
                width = w.atom_args().first().and_then(|s| parse_num(s)).map(|v| scale(v, res));
            }
            if let Some(c) = rule.child("clearance") {
                clearance = c.atom_args().first().and_then(|s| parse_num(s)).map(|v| scale(v, res));
            }
        }
        out.push(fr_board::NetClass { name, nets, width, clearance });
    }
    out
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

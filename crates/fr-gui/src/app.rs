//! The freerouting-rs egui application.
//!
//! Features: open a DSN (in-app file browser + typed path), render the board (outline,
//! pads, per-layer traces, vias, and a ratsnest of unrouted nets) with pan/zoom and
//! zoom-to-fit, a routing-parameters config panel, manual commands (route / clear),
//! layer visibility + a color legend, net highlight, and an incompletes readout.

use std::path::PathBuf;

use eframe::egui;
use egui::{Color32, Pos2, Sense, Stroke};
use fr_board::Board;
use fr_geometry::Point;
use fr_dsn::{read_board, write_rte, write_ses};
use fr_engine::{net_ratsnest, route_board, AngleRestriction, InteractiveRouter, RouteOptions, RouteReport};

use crate::view::ViewTransform;

pub struct App {
    board: Option<Board>,
    view: Option<ViewTransform>,
    refit: bool,
    layer_visible: Vec<bool>,
    status: String,
    last_report: Option<RouteReport>,
    loaded_path: Option<PathBuf>,

    // routing parameters (config panel)
    opt_max_time: u64,
    opt_threads: usize,
    opt_width_mil: f64,
    opt_clearance_mil: f64,
    opt_max_layers: usize,

    // view toggles
    show_ratsnest: bool,
    show_pads: bool,
    highlight_net: Option<usize>,

    // selection (click-picked item)
    selected: Option<crate::picking::Pick>,

    // interactive manual routing
    route_mode: bool,
    router: Option<InteractiveRouter>,
    snap_angle: AngleRestriction,
    allow_vias: bool,
    shove: bool,
    active_layer: usize,

    // file browser
    show_browser: bool,
    browser_dir: PathBuf,
    path_input: String,
}

impl Default for App {
    fn default() -> Self {
        App {
            board: None,
            view: None,
            refit: false,
            layer_visible: Vec::new(),
            status: "Open a Specctra .dsn (Browse… or type a path) to begin.".into(),
            last_report: None,
            loaded_path: None,
            opt_max_time: 0,
            opt_threads: 0,
            opt_width_mil: 0.0,
            opt_clearance_mil: 0.0,
            opt_max_layers: 0,
            show_ratsnest: true,
            show_pads: true,
            highlight_net: None,
            selected: None,
            route_mode: false,
            router: None,
            snap_angle: AngleRestriction::None,
            allow_vias: true,
            shove: false,
            active_layer: 0,
            show_browser: false,
            browser_dir: std::env::current_dir().unwrap_or_else(|_| PathBuf::from("/")),
            path_input: String::new(),
        }
    }
}

impl App {
    pub fn load_path(&mut self, path: PathBuf) {
        match std::fs::read_to_string(&path) {
            Ok(src) => {
                let (board, warnings) = read_board(&src);
                self.layer_visible = vec![true; board.layer_count().max(1)];
                self.status = format!(
                    "Loaded {}: {} layers, {} nets, {} components ({} warnings)",
                    board.name, board.layer_count(), board.nets.len(),
                    board.components.len(), warnings.len()
                );
                self.refit = true;
                self.last_report = None;
                self.highlight_net = None;
                self.selected = None;
                self.router = None;
                self.route_mode = false;
                self.board = Some(board);
                self.path_input = path.display().to_string();
                self.loaded_path = Some(path);
                self.show_browser = false;
            }
            Err(e) => self.status = format!("Failed to read '{}': {e}", path.display()),
        }
    }

    fn route(&mut self) {
        let Some(board) = self.board.as_mut() else { return };
        board.traces.clear();
        board.vias.clear();
        let per_unit = board.resolution.per_unit as f64;
        let opts = RouteOptions {
            max_time_secs: self.opt_max_time,
            threads: self.opt_threads,
            seed: 1,
            width: (self.opt_width_mil * per_unit) as i64,
            clearance: (self.opt_clearance_mil * per_unit) as i64,
            max_layers: self.opt_max_layers,
        };
        let report = route_board(board, &opts);
        self.selected = None; // trace/via indices changed
        self.router = None; // board geometry changed; rebuild router lazily
        self.status = format!(
            "Routed {}/{} nets, {} traces, {} vias ({} incomplete)",
            report.nets_completed, report.nets_total,
            board.traces.len(), board.vias.len(), report.unrouted_nets.len()
        );
        self.last_report = Some(report);
    }

    /// Ensure the interactive router exists (rebuilt from the current board on demand).
    fn ensure_router(&mut self) {
        if self.router.is_none() {
            if let Some(board) = self.board.as_mut() {
                let mut r = InteractiveRouter::new(board);
                r.set_angle(self.snap_angle);
                r.set_allow_vias(self.allow_vias);
                self.router = Some(r);
            }
        }
    }

    /// Handle a click in route mode at board point `p`: if no route is in progress, start
    /// one at `p`; otherwise commit a segment to `p`. Both use the active layer.
    fn route_click(&mut self, p: Point) {
        self.ensure_router();
        let layer = self.active_layer;
        let net = self.highlight_net.map(|n| n as u32).unwrap_or(0);
        let Some(router) = self.router.as_mut() else { return };
        if !router.has_start() {
            router.begin(p, layer, net);
            self.status = format!(
                "Manual route started on layer {layer}, net {net}. Click to place; right-click/Esc to finish."
            );
        } else if let Some(board) = self.board.as_mut() {
            if self.shove {
                let out = router.commit_shove(board, p, layer);
                self.last_report = None;
                self.status = if out.committed {
                    format!("Placed segment (shove: {} rerouted, {} dropped).", out.rerouted, out.dropped)
                } else {
                    "No route even with shove (try another path or layer).".into()
                };
            } else if router.commit(board, p, layer) {
                self.last_report = None;
                self.status = "Placed a manual route segment.".into();
            } else {
                self.status = "No clear route to that point (enable Shove, or try another path/layer).".into();
            }
        }
    }

    /// Finish/cancel the in-progress manual route.
    fn route_finish(&mut self) {
        if let Some(r) = self.router.as_mut() {
            r.cancel();
        }
        self.status = "Manual route finished.".into();
    }

    fn clear_routes(&mut self) {
        if let Some(board) = self.board.as_mut() {
            board.traces.clear();
            board.vias.clear();
            self.last_report = None;
            self.selected = None;
            self.router = None;
            self.status = "Cleared all routed traces and vias.".into();
        }
    }

    fn export(&mut self, ext: &str) {
        let Some(board) = &self.board else { return };
        let path = self
            .loaded_path
            .clone()
            .unwrap_or_else(|| PathBuf::from("freerouting-rs-output.dsn"))
            .with_extension(ext);
        let out = if ext == "ses" { write_ses(board) } else { write_rte(board) };
        match std::fs::write(&path, out) {
            Ok(()) => self.status = format!("Exported {}", path.display()),
            Err(e) => self.status = format!("Export failed: {e}"),
        }
    }

    fn layer_color(layer: usize) -> Color32 {
        const PALETTE: [Color32; 6] = [
            Color32::from_rgb(220, 60, 60),
            Color32::from_rgb(60, 140, 220),
            Color32::from_rgb(80, 200, 100),
            Color32::from_rgb(220, 180, 60),
            Color32::from_rgb(200, 100, 220),
            Color32::from_rgb(60, 200, 200),
        ];
        PALETTE[layer % PALETTE.len()]
    }

    fn draw_board(&mut self, ui: &mut egui::Ui) {
        let Some(board) = &self.board else {
            ui.centered_and_justified(|ui| { ui.label("No board loaded."); });
            return;
        };
        let (resp, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let screen = resp.rect;
        // Dark neutral backdrop, clearly distinct from the (greener) board substrate.
        painter.rect_filled(screen, 0.0, Color32::from_rgb(12, 12, 16));

        if self.view.is_none() || self.refit {
            let bounds = board
                .outline_box()
                .or_else(|| fr_geometry::IntBox::bound(board.pins.iter().map(|p| p.location)))
                .unwrap_or(fr_geometry::IntBox::new(0, 0, 1, 1));
            self.view = Some(ViewTransform::fit(bounds, screen));
            self.refit = false;
        }
        let view = self.view.as_mut().unwrap();

        if resp.dragged() {
            view.pan_pixels(resp.drag_delta());
        }
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll.abs() > 0.0 {
            let anchor = resp.hover_pos().unwrap_or(screen.center());
            view.zoom_at((scroll as f64 * 0.002).exp(), anchor, screen);
        }

        // Hit-test under the cursor (within a few-pixel tolerance, converted to board
        // units). Used for the hover tooltip and click selection. A drag is a pan, not a
        // pick, so only treat a non-drag click as a selection.
        let view_ro = *view;
        let tol_board = 5.0 / view_ro.scale.max(1e-9);
        let hover_pick = resp.hover_pos().and_then(|hp| {
            let q = view_ro.to_board(hp, screen);
            crate::picking::pick_at(board, q, tol_board, &self.layer_visible)
        });
        // In route mode, a left click routes (start/commit) instead of selecting; a right
        // click or Esc finishes. Capture intents now; apply after the board borrow ends.
        let cursor_board = resp.hover_pos().map(|hp| view_ro.to_board(hp, screen));
        let esc = ui.input(|i| i.key_pressed(egui::Key::Escape));
        let route_click_at: Option<Point> = if self.route_mode && resp.clicked() {
            cursor_board
        } else {
            None
        };
        let route_finish = self.route_mode && (resp.secondary_clicked() || esc);
        let new_selection: Option<Option<crate::picking::Pick>> =
            if resp.clicked() && !self.route_mode { Some(hover_pick) } else { None };

        // board outline: filled substrate (concave-safe via ear-clipping) + edge stroke,
        // giving clear board/background contrast. The fill is built as an explicit Mesh
        // (winding-independent) rather than convex_polygon, because to_screen flips Y and
        // egui's convex-polygon fill is winding-sensitive (it can drop reversed triangles
        // on some backends). The Mesh rasterizes regardless of vertex order.
        if board.outline.len() >= 3 {
            let board_fill = Color32::from_rgb(22, 64, 50); // dark PCB green-teal
            let mut mesh = egui::epaint::Mesh::default();
            for tri in crate::padgeom::triangulate(&board.outline) {
                let base = mesh.vertices.len() as u32;
                for &vi in &tri {
                    mesh.colored_vertex(view.to_screen(board.outline[vi], screen), board_fill);
                }
                mesh.add_triangle(base, base + 1, base + 2);
            }
            if !mesh.is_empty() {
                painter.add(egui::Shape::mesh(mesh));
            }
            // bright, bold edge so the board boundary is unmistakable.
            let edge: Vec<Pos2> = board.outline.iter().map(|&p| view.to_screen(p, screen)).collect();
            for i in 0..edge.len() {
                painter.line_segment(
                    [edge[i], edge[(i + 1) % edge.len()]],
                    Stroke::new(2.5, Color32::from_rgb(150, 230, 200)),
                );
            }
        }

        // pads: real per-layer copper geometry (circle radius / convex polygon), scaled.
        if self.show_pads {
            for pin in &board.pins {
                let Some(shape) = crate::padgeom::pin_pad_shape(board, pin) else { continue };
                let hl = self.highlight_net.is_some() && pin.net == self.highlight_net;
                let col = if hl {
                    Color32::from_rgb(245, 230, 130)
                } else {
                    Color32::from_rgb(170, 160, 90) // brass/pad color, distinct from traces
                };
                match shape {
                    crate::padgeom::PadDraw::Circle { center, radius } => {
                        let c = view.to_screen(center, screen);
                        let r = ((radius as f64 * view.scale) as f32).max(1.0);
                        painter.circle_filled(c, r, col);
                    }
                    crate::padgeom::PadDraw::Poly(verts) => {
                        // Mesh fill (winding-independent under the Y-flip), fan-triangulated
                        // since pad polygons are convex.
                        let pts: Vec<Pos2> = verts.iter().map(|&p| view.to_screen(p, screen)).collect();
                        if pts.len() >= 3 {
                            let mut mesh = egui::epaint::Mesh::default();
                            for &p in &pts {
                                mesh.colored_vertex(p, col);
                            }
                            for i in 1..(pts.len() as u32 - 1) {
                                mesh.add_triangle(0, i, i + 1);
                            }
                            painter.add(egui::Shape::mesh(mesh));
                        }
                    }
                }
            }
        }

        // ratsnest of unrouted nets (thin gray lines between unconnected pins)
        if self.show_ratsnest {
            if let Some(rep) = &self.last_report {
                for &nid in &rep.unrouted_nets {
                    for (a, b) in net_ratsnest(board, nid) {
                        painter.line_segment(
                            [view.to_screen(a, screen), view.to_screen(b, screen)],
                            Stroke::new(0.5, Color32::from_rgb(90, 90, 70)),
                        );
                    }
                }
            }
        }

        // traces
        for t in &board.traces {
            if t.layer >= self.layer_visible.len() || !self.layer_visible[t.layer] {
                continue;
            }
            let dim = self.highlight_net.is_some() && t.net != self.highlight_net;
            let mut col = Self::layer_color(t.layer);
            if dim {
                col = col.gamma_multiply(0.25);
            }
            let w = (((t.width as f64) * view.scale).max(1.0) as f32).min(6.0);
            for seg in t.corners.windows(2) {
                painter.line_segment(
                    [view.to_screen(seg[0], screen), view.to_screen(seg[1], screen)],
                    Stroke::new(w, col),
                );
            }
        }

        // vias
        for v in &board.vias {
            let c = view.to_screen(v.location, screen);
            let dim = self.highlight_net.is_some() && v.net != self.highlight_net;
            let col = if dim { Color32::from_gray(120) } else { Color32::WHITE };
            painter.circle_stroke(c, 3.0, Stroke::new(1.5, col));
        }

        // selection highlight (cyan): outline the currently-selected item.
        if let Some(sel) = self.selected {
            Self::stroke_pick(&painter, board, &view_ro, screen, sel,
                Stroke::new(2.5, Color32::from_rgb(0, 230, 255)));
        }
        // hover highlight (faint white) when not the same as the selection.
        if let Some(hp) = hover_pick {
            if Some(hp) != self.selected {
                Self::stroke_pick(&painter, board, &view_ro, screen, hp,
                    Stroke::new(1.5, Color32::from_rgba_unmultiplied(255, 255, 255, 160)));
            }
        }

        // Manual-route preview: from the in-progress start to the cursor, draw the would-be
        // route (green = clear, red = no route). Also mark the route start anchor.
        if self.route_mode {
            if let (Some(router), Some(cur)) = (self.router.as_ref(), cursor_board) {
                if let Some((sp, _sl)) = router.start_point() {
                    painter.circle_stroke(
                        view_ro.to_screen(sp, screen), 5.0,
                        Stroke::new(2.0, Color32::from_rgb(0, 255, 120)),
                    );
                    match router.preview(cur, self.active_layer) {
                        Some(conn) => {
                            for t in &conn.traces {
                                for seg in t.corners.windows(2) {
                                    painter.line_segment(
                                        [view_ro.to_screen(seg[0], screen), view_ro.to_screen(seg[1], screen)],
                                        Stroke::new(2.0, Color32::from_rgb(0, 255, 120)),
                                    );
                                }
                            }
                            for v in &conn.vias {
                                painter.circle_filled(view_ro.to_screen(v.location, screen), 4.0,
                                    Color32::from_rgb(0, 255, 120));
                            }
                        }
                        None => {
                            // no clear route: draw a faint red rubber-band to the cursor
                            painter.line_segment(
                                [view_ro.to_screen(sp, screen), view_ro.to_screen(cur, screen)],
                                Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 80, 80, 160)),
                            );
                        }
                    }
                }
            }
        }

        // hover tooltip with item info (suppressed in route mode to avoid clutter)
        if !self.route_mode {
            if let Some(hp) = hover_pick {
                let text = Self::pick_info(board, hp);
                egui::show_tooltip_at_pointer(ui.ctx(), ui.layer_id(), egui::Id::new("pick_tip"), |ui| {
                    ui.label(text);
                });
            }
        }

        if let Some(sel) = new_selection {
            self.selected = sel;
        }
        // apply captured route-mode intents (board borrow has ended)
        if route_finish {
            self.route_finish();
        } else if let Some(p) = route_click_at {
            self.route_click(p);
        }
    }

    /// Stroke an outline around a picked item for selection/hover feedback.
    fn stroke_pick(
        painter: &egui::Painter,
        board: &Board,
        view: &ViewTransform,
        screen: egui::Rect,
        pick: crate::picking::Pick,
        stroke: Stroke,
    ) {
        use crate::picking::Pick;
        match pick {
            Pick::Trace { index } => {
                if let Some(t) = board.traces.get(index) {
                    for seg in t.corners.windows(2) {
                        painter.line_segment(
                            [view.to_screen(seg[0], screen), view.to_screen(seg[1], screen)],
                            stroke,
                        );
                    }
                }
            }
            Pick::Via { index } => {
                if let Some(v) = board.vias.get(index) {
                    painter.circle_stroke(view.to_screen(v.location, screen), 5.0, stroke);
                }
            }
            Pick::Pad { pin_index } => {
                if let Some(pin) = board.pins.get(pin_index) {
                    if let Some(shape) = crate::padgeom::pin_pad_shape(board, pin) {
                        match shape {
                            crate::padgeom::PadDraw::Circle { center, radius } => {
                                let r = ((radius as f64 * view.scale) as f32).max(2.0) + 1.5;
                                painter.circle_stroke(view.to_screen(center, screen), r, stroke);
                            }
                            crate::padgeom::PadDraw::Poly(verts) => {
                                let pts: Vec<Pos2> = verts.iter().map(|&p| view.to_screen(p, screen)).collect();
                                for i in 0..pts.len() {
                                    painter.line_segment([pts[i], pts[(i + 1) % pts.len()]], stroke);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// Human-readable info for a picked item (net, layer, width, coords).
    fn pick_info(board: &Board, pick: crate::picking::Pick) -> String {
        use crate::picking::Pick;
        let per_unit = board.resolution.per_unit as f64;
        let net_name = |net: Option<usize>| -> String {
            net.and_then(|n| board.nets.get(n)).map(|x| x.name.clone())
                .unwrap_or_else(|| "<no net>".into())
        };
        match pick {
            Pick::Trace { index } => {
                let Some(t) = board.traces.get(index) else { return "trace".into() };
                let layer = board.layers.layers().get(t.layer).map(|l| l.name.as_str()).unwrap_or("?");
                let len: f64 = t.corners.windows(2)
                    .map(|s| ((s[1].x - s[0].x) as f64).hypot((s[1].y - s[0].y) as f64))
                    .sum();
                format!(
                    "Trace\nnet: {}\nlayer: {}\nwidth: {:.1} mil\nlength: {:.1} mil\ncorners: {}",
                    net_name(t.net), layer, t.width as f64 / per_unit, len / per_unit, t.corners.len()
                )
            }
            Pick::Via { index } => {
                let Some(v) = board.vias.get(index) else { return "via".into() };
                let ps = board.padstacks.get(v.padstack).map(|p| p.name.as_str()).unwrap_or("?");
                format!(
                    "Via\nnet: {}\npadstack: {}\nat: ({:.1}, {:.1}) mil",
                    net_name(v.net), ps,
                    v.location.x as f64 / per_unit, v.location.y as f64 / per_unit
                )
            }
            Pick::Pad { pin_index } => {
                let Some(pin) = board.pins.get(pin_index) else { return "pad".into() };
                let ps = board.padstacks.get(pin.padstack).map(|p| p.name.as_str()).unwrap_or("?");
                format!(
                    "Pad {}-{}\nnet: {}\npadstack: {}\nat: ({:.1}, {:.1}) mil",
                    pin.component, pin.name, net_name(pin.net), ps,
                    pin.location.x as f64 / per_unit, pin.location.y as f64 / per_unit
                )
            }
        }
    }

    fn file_browser(&mut self, ctx: &egui::Context) {
        if !self.show_browser {
            return;
        }
        let mut open = true;
        let mut to_load: Option<PathBuf> = None;
        egui::Window::new("Open DSN")
            .open(&mut open)
            .resizable(true)
            .default_size([560.0, 420.0])
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label("Dir:");
                    ui.monospace(self.browser_dir.display().to_string());
                    if ui.button("⬆ up").clicked() {
                        if let Some(p) = self.browser_dir.parent() {
                            self.browser_dir = p.to_path_buf();
                        }
                    }
                });
                ui.separator();
                egui::ScrollArea::vertical().max_height(320.0).show(ui, |ui| {
                    // directories first, then .dsn files
                    let mut entries: Vec<(bool, String, PathBuf)> = Vec::new();
                    if let Ok(rd) = std::fs::read_dir(&self.browser_dir) {
                        for e in rd.flatten() {
                            let p = e.path();
                            let is_dir = p.is_dir();
                            let name = e.file_name().to_string_lossy().to_string();
                            let is_dsn = p.extension().map(|x| x.eq_ignore_ascii_case("dsn")).unwrap_or(false);
                            if is_dir || is_dsn {
                                entries.push((is_dir, name, p));
                            }
                        }
                    }
                    entries.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
                    for (is_dir, name, path) in entries {
                        let label = if is_dir { format!("📁 {name}") } else { format!("📄 {name}") };
                        if ui.selectable_label(false, label).clicked() {
                            if is_dir {
                                self.browser_dir = path;
                            } else {
                                to_load = Some(path);
                            }
                        }
                    }
                });
            });
        if let Some(p) = to_load {
            self.load_path(p);
        }
        if !open {
            self.show_browser = false;
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let has_board = self.board.is_some();

        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                if ui.button("Browse…").clicked() {
                    if let Some(p) = self.loaded_path.clone().and_then(|p| p.parent().map(|x| x.to_path_buf())) {
                        self.browser_dir = p;
                    }
                    self.show_browser = true;
                }
                ui.add(egui::TextEdit::singleline(&mut self.path_input).desired_width(360.0).hint_text("/path/to/board.dsn"));
                if ui.button("Open").clicked() {
                    let p = self.path_input.trim().to_string();
                    if !p.is_empty() { self.load_path(PathBuf::from(p)); }
                }
                ui.separator();
                if ui.add_enabled(has_board, egui::Button::new("▶ Route")).clicked() {
                    self.route();
                }
                if ui.add_enabled(has_board, egui::Button::new("Clear")).clicked() {
                    self.clear_routes();
                }
                if ui.add_enabled(has_board, egui::Button::new("Fit")).clicked() {
                    self.refit = true;
                }
                ui.separator();
                if ui.add_enabled(has_board, egui::Button::new("Export RTE")).clicked() {
                    self.export("rte");
                }
                if ui.add_enabled(has_board, egui::Button::new("Export SES")).clicked() {
                    self.export("ses");
                }
            });
        });

        egui::SidePanel::left("panel").resizable(true).default_width(220.0).show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.collapsing("Routing parameters", |ui| {
                    ui.horizontal(|ui| { ui.label("Max time (s, 0=∞):"); ui.add(egui::DragValue::new(&mut self.opt_max_time).range(0..=3600)); });
                    ui.horizontal(|ui| { ui.label("Threads (0=auto):"); ui.add(egui::DragValue::new(&mut self.opt_threads).range(0..=64)); });
                    ui.horizontal(|ui| { ui.label("Width (mil, 0=rule):"); ui.add(egui::DragValue::new(&mut self.opt_width_mil).range(0.0..=200.0).speed(0.5)); });
                    ui.horizontal(|ui| { ui.label("Clearance (mil, 0=rule):"); ui.add(egui::DragValue::new(&mut self.opt_clearance_mil).range(0.0..=200.0).speed(0.5)); });
                    ui.horizontal(|ui| { ui.label("Max layers (0=all):"); ui.add(egui::DragValue::new(&mut self.opt_max_layers).range(0..=32)); });
                });

                ui.collapsing("View", |ui| {
                    ui.checkbox(&mut self.show_ratsnest, "Ratsnest (unrouted)");
                    ui.checkbox(&mut self.show_pads, "Pads");
                    if self.highlight_net.is_some() && ui.button("Clear highlight").clicked() {
                        self.highlight_net = None;
                    }
                });

                // Manual routing (free-angle room/door model): toggle route mode, snap
                // angle, vias, and the active layer. In route mode: click to start a route
                // at the cursor, click again to place segments, right-click/Esc to finish.
                ui.collapsing("Manual route", |ui| {
                    let layer_count = self.board.as_ref().map(|b| b.layer_count().max(1)).unwrap_or(1);
                    if ui.checkbox(&mut self.route_mode, "Route mode (click to draw)").changed()
                        && self.route_mode
                    {
                        self.ensure_router();
                    }
                    ui.label("Snap angle:");
                    let mut changed = false;
                    ui.horizontal(|ui| {
                        changed |= ui.selectable_value(&mut self.snap_angle, AngleRestriction::None, "Any").clicked();
                        changed |= ui.selectable_value(&mut self.snap_angle, AngleRestriction::FortyFive, "45°").clicked();
                        changed |= ui.selectable_value(&mut self.snap_angle, AngleRestriction::Ninety, "90°").clicked();
                    });
                    let vias_changed = ui.checkbox(&mut self.allow_vias, "Allow vias (layer changes)").changed();
                    ui.checkbox(&mut self.shove, "Shove (rip-up & reroute blockers)");
                    ui.horizontal(|ui| {
                        ui.label("Active layer:");
                        ui.add(egui::DragValue::new(&mut self.active_layer).range(0..=(layer_count.saturating_sub(1))));
                    });
                    if changed || vias_changed {
                        if let Some(r) = self.router.as_mut() {
                            r.set_angle(self.snap_angle);
                            r.set_allow_vias(self.allow_vias);
                        }
                    }
                    ui.label("Net: highlighted net, or 0. Pick a net under 'Nets' first to route it.");
                });

                // Selection info: details of the clicked item, with actions.
                let mut do_highlight_net: Option<usize> = None;
                let mut do_clear_selection = false;
                if let (Some(board), Some(sel)) = (&self.board, self.selected) {
                    let info = Self::pick_info(board, sel);
                    let net = match sel {
                        crate::picking::Pick::Trace { index } =>
                            board.traces.get(index).and_then(|t| t.net),
                        crate::picking::Pick::Via { index } =>
                            board.vias.get(index).and_then(|v| v.net),
                        crate::picking::Pick::Pad { pin_index } =>
                            board.pins.get(pin_index).and_then(|p| p.net),
                    };
                    ui.collapsing("Selection", |ui| {
                        ui.label(info);
                        ui.horizontal(|ui| {
                            if net.is_some() && ui.button("Highlight net").clicked() {
                                do_highlight_net = net;
                            }
                            if ui.button("Clear selection").clicked() {
                                do_clear_selection = true;
                            }
                        });
                    });
                }
                if do_highlight_net.is_some() {
                    self.highlight_net = do_highlight_net;
                }
                if do_clear_selection {
                    self.selected = None;
                }

                ui.collapsing("Layers", |ui| {
                    if let Some(board) = &self.board {
                        for (i, layer) in board.layers.layers().iter().enumerate() {
                            if i < self.layer_visible.len() {
                                ui.horizontal(|ui| {
                                    ui.checkbox(&mut self.layer_visible[i], "");
                                    let (rect, _) = ui.allocate_exact_size(egui::vec2(12.0, 12.0), Sense::hover());
                                    ui.painter().rect_filled(rect, 2.0, App::layer_color(i));
                                    ui.label(&layer.name);
                                });
                            }
                        }
                    }
                });

                if let (Some(board), true) = (&self.board, self.highlight_net.is_none()) {
                    ui.collapsing("Nets", |ui| {
                        egui::ScrollArea::vertical().max_height(200.0).id_source("nets").show(ui, |ui| {
                            for (id, net) in board.nets.iter() {
                                if ui.selectable_label(false, &net.name).clicked() {
                                    self.highlight_net = Some(id);
                                }
                            }
                        });
                    });
                }

                if let Some(rep) = &self.last_report {
                    ui.separator();
                    ui.label(format!("Routed: {}/{}", rep.nets_completed, rep.nets_total));
                    ui.label(format!("Incomplete: {}", rep.unrouted_nets.len()));
                }
            });
        });

        egui::TopBottomPanel::bottom("status").show(ctx, |ui| {
            ui.label(&self.status);
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            self.draw_board(ui);
        });

        self.file_browser(ctx);
    }
}

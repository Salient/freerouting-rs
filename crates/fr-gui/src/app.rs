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
use fr_dsn::{read_board, write_rte, write_ses};
use fr_engine::{net_ratsnest, route_board, RouteOptions, RouteReport};

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
        self.status = format!(
            "Routed {}/{} nets, {} traces, {} vias ({} incomplete)",
            report.nets_completed, report.nets_total,
            board.traces.len(), board.vias.len(), report.unrouted_nets.len()
        );
        self.last_report = Some(report);
    }

    fn clear_routes(&mut self) {
        if let Some(board) = self.board.as_mut() {
            board.traces.clear();
            board.vias.clear();
            self.last_report = None;
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
        painter.rect_filled(screen, 0.0, Color32::from_rgb(18, 20, 18));

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

        // board outline
        if board.outline.len() >= 2 {
            let pts: Vec<Pos2> = board.outline.iter().map(|&p| view.to_screen(p, screen)).collect();
            for i in 0..pts.len() {
                painter.line_segment([pts[i], pts[(i + 1) % pts.len()]], Stroke::new(1.5, Color32::from_gray(150)));
            }
        }

        // pads
        if self.show_pads {
            for pin in &board.pins {
                let c = view.to_screen(pin.location, screen);
                if screen.contains(c) {
                    let hl = self.highlight_net.is_some() && pin.net == self.highlight_net;
                    let col = if hl { Color32::WHITE } else { Color32::from_gray(105) };
                    painter.circle_filled(c, if hl { 2.5 } else { 1.5 }, col);
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

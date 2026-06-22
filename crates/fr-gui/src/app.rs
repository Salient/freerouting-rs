//! The freerouting-rs egui application: open a DSN, render the board (outline, pads,
//! traces, vias) with pan/zoom, toggle layer visibility, run the autorouter, and export
//! an Altium-importable RTE/SES.

use std::path::PathBuf;

use eframe::egui;
use egui::{Color32, Pos2, Sense, Stroke};
use fr_board::Board;
use fr_dsn::{read_board, write_rte, write_ses};
use fr_engine::{route_board, RouteOptions, RouteReport};

use crate::view::ViewTransform;

pub struct App {
    board: Option<Board>,
    view: Option<ViewTransform>,
    layer_visible: Vec<bool>,
    status: String,
    last_report: Option<RouteReport>,
    max_time: u64,
    loaded_path: Option<PathBuf>,
}

impl Default for App {
    fn default() -> Self {
        App {
            board: None,
            view: None,
            layer_visible: Vec::new(),
            status: "Open a Specctra .dsn to begin.".into(),
            last_report: None,
            max_time: 30,
            loaded_path: None,
        }
    }
}

impl App {
    /// Load a board from a DSN path (used by both the GUI button and headless tests).
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
                self.view = None; // refit on next paint
                self.board = Some(board);
                self.loaded_path = Some(path);
            }
            Err(e) => self.status = format!("Failed to read file: {e}"),
        }
    }

    pub fn route(&mut self) {
        if let Some(board) = self.board.as_mut() {
            board.traces.clear();
            board.vias.clear();
            let opts = RouteOptions { max_time_secs: self.max_time, threads: 0, seed: 1 };
            let report = route_board(board, &opts);
            self.status = format!(
                "Routed {}/{} nets, {} traces, {} vias",
                report.nets_completed, report.nets_total, board.traces.len(), board.vias.len()
            );
            self.last_report = Some(report);
        }
    }

    pub fn export(&mut self, path: PathBuf) {
        if let Some(board) = &self.board {
            let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("rte").to_lowercase();
            let out = if ext == "ses" { write_ses(board) } else { write_rte(board) };
            match std::fs::write(&path, out) {
                Ok(()) => self.status = format!("Exported {}", path.display()),
                Err(e) => self.status = format!("Export failed: {e}"),
            }
        }
    }

    fn layer_color(layer: usize) -> Color32 {
        const PALETTE: [Color32; 6] = [
            Color32::from_rgb(220, 60, 60),   // top - red
            Color32::from_rgb(60, 140, 220),  // mid1 - blue
            Color32::from_rgb(80, 200, 100),  // mid2 - green
            Color32::from_rgb(220, 180, 60),  // mid3 - yellow
            Color32::from_rgb(200, 100, 220), // mid4 - purple
            Color32::from_rgb(60, 200, 200),  // bottom - cyan
        ];
        PALETTE[layer % PALETTE.len()]
    }

    fn draw_board(&mut self, ui: &mut egui::Ui) {
        let Some(board) = &self.board else { return };
        let (resp, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let screen = resp.rect;
        painter.rect_filled(screen, 0.0, Color32::from_rgb(20, 24, 20));

        // initialize / refit view
        if self.view.is_none() {
            let bounds = board
                .outline_box()
                .or_else(|| fr_geometry::IntBox::bound(board.pins.iter().map(|p| p.location)))
                .unwrap_or(fr_geometry::IntBox::new(0, 0, 1, 1));
            self.view = Some(ViewTransform::fit(bounds, screen));
        }
        let view = self.view.as_mut().unwrap();

        // interaction: drag to pan, scroll to zoom
        if resp.dragged() {
            view.pan_pixels(resp.drag_delta());
        }
        let scroll = ui.input(|i| i.smooth_scroll_delta.y);
        if scroll.abs() > 0.0 {
            let anchor = resp.hover_pos().unwrap_or(screen.center());
            let factor = (scroll as f64 * 0.002).exp();
            view.zoom_at(factor, anchor, screen);
        }

        // board outline
        if board.outline.len() >= 2 {
            let pts: Vec<Pos2> = board.outline.iter().map(|&p| view.to_screen(p, screen)).collect();
            for i in 0..pts.len() {
                let a = pts[i];
                let b = pts[(i + 1) % pts.len()];
                painter.line_segment([a, b], Stroke::new(1.5, Color32::from_gray(160)));
            }
        }

        // pads (small dots, gray)
        for pin in &board.pins {
            let c = view.to_screen(pin.location, screen);
            if screen.contains(c) {
                painter.circle_filled(c, 1.5, Color32::from_gray(110));
            }
        }

        // traces, colored per visible layer
        for t in &board.traces {
            if t.layer >= self.layer_visible.len() || !self.layer_visible[t.layer] {
                continue;
            }
            let col = Self::layer_color(t.layer);
            let w = ((t.width as f64) * view.scale).max(1.0) as f32;
            for seg in t.corners.windows(2) {
                let a = view.to_screen(seg[0], screen);
                let b = view.to_screen(seg[1], screen);
                painter.line_segment([a, b], Stroke::new(w.min(6.0), col));
            }
        }

        // vias (white rings)
        for v in &board.vias {
            let c = view.to_screen(v.location, screen);
            painter.circle_stroke(c, 3.0, Stroke::new(1.5, Color32::WHITE));
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                if ui.button("Open DSN…").clicked() {
                    if let Some(path) = rfd::FileDialog::new().add_filter("Specctra DSN", &["dsn"]).pick_file() {
                        self.load_path(path);
                    }
                }
                ui.separator();
                ui.label("Max time (s):");
                ui.add(egui::DragValue::new(&mut self.max_time).range(0..=600));
                if ui.add_enabled(self.board.is_some(), egui::Button::new("Route")).clicked() {
                    self.route();
                }
                ui.separator();
                if ui.add_enabled(self.board.is_some(), egui::Button::new("Export RTE…")).clicked() {
                    if let Some(path) = rfd::FileDialog::new().add_filter("Specctra Route", &["rte"]).save_file() {
                        self.export(path);
                    }
                }
                if ui.add_enabled(self.board.is_some(), egui::Button::new("Export SES…")).clicked() {
                    if let Some(path) = rfd::FileDialog::new().add_filter("Specctra Session", &["ses"]).save_file() {
                        self.export(path);
                    }
                }
            });
        });

        egui::SidePanel::left("layers").resizable(false).show(ctx, |ui| {
            ui.heading("Layers");
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
            } else {
                ui.label("(no board)");
            }
        });

        egui::TopBottomPanel::bottom("status").show(ctx, |ui| {
            ui.label(&self.status);
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            self.draw_board(ui);
        });
    }
}

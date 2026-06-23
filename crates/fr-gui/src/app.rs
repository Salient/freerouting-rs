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
    warnings: Vec<String>,
    show_warnings: bool,

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

    // layer emphasis: fade non-active layers (opacity + desaturation) so the active layer
    // stands out. Configurable; on by default.
    fade_inactive: bool,

    // show the clearance keepout boundary around obstacles (copper + design clearance).
    show_clearance: bool,
    // show DRC violation markers on the canvas
    show_violations: bool,
    // show keepout regions
    show_keepouts: bool,

    // info windows
    show_incompletes_win: bool,
    show_violations_win: bool,
    show_stats_win: bool,
    show_components_win: bool,
    show_nets_win: bool,
    show_classes_win: bool,
    show_help_win: bool,
    comp_filter: String,
    net_filter: String,
    // cached DRC violations (recomputed on demand)
    violations: Vec<fr_engine::Violation>,
    // request to recenter the view on a board point (from a list click)
    goto_point: Option<Point>,
    // cursor position in board units (for the status bar)
    cursor_pos: Option<Point>,

    // file browser
    show_browser: bool,
    browser_dir: PathBuf,
    path_input: String,

    // unlimited undo/redo of board mutations (snapshots of traces+vias).
    undo_stack: Vec<BoardSnapshot>,
    redo_stack: Vec<BoardSnapshot>,
}

/// A snapshot of the routable geometry for undo/redo (the only thing routing mutates).
#[derive(Clone)]
struct BoardSnapshot {
    traces: Vec<fr_board::Trace>,
    vias: Vec<fr_board::Via>,
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
            warnings: Vec::new(),
            show_warnings: false,
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
            fade_inactive: true,
            show_clearance: false,
            show_violations: false,
            show_keepouts: true,
            show_incompletes_win: false,
            show_violations_win: false,
            show_stats_win: false,
            show_components_win: false,
            show_nets_win: false,
            show_classes_win: false,
            show_help_win: false,
            comp_filter: String::new(),
            net_filter: String::new(),
            violations: Vec::new(),
            goto_point: None,
            cursor_pos: None,
            show_browser: false,
            browser_dir: std::env::current_dir().unwrap_or_else(|_| PathBuf::from("/")),
            path_input: String::new(),
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
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
                self.warnings = warnings;
                self.active_layer = self.active_layer.min(board.layer_count().saturating_sub(1));
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

    /// Push the current board geometry onto the undo stack (clearing redo). Call BEFORE a
    /// mutation. No-op if there's no board.
    fn push_undo(&mut self) {
        if let Some(board) = self.board.as_ref() {
            self.undo_stack.push(BoardSnapshot {
                traces: board.traces.clone(),
                vias: board.vias.clone(),
            });
            self.redo_stack.clear();
        }
    }

    fn undo(&mut self) {
        let Some(board) = self.board.as_mut() else { return };
        if let Some(prev) = self.undo_stack.pop() {
            self.redo_stack.push(BoardSnapshot { traces: board.traces.clone(), vias: board.vias.clone() });
            board.traces = prev.traces;
            board.vias = prev.vias;
            self.router = None;
            self.selected = None;
            self.last_report = None;
            self.status = format!("Undo ({} more).", self.undo_stack.len());
        } else {
            self.status = "Nothing to undo.".into();
        }
    }

    fn redo(&mut self) {
        let Some(board) = self.board.as_mut() else { return };
        if let Some(next) = self.redo_stack.pop() {
            self.undo_stack.push(BoardSnapshot { traces: board.traces.clone(), vias: board.vias.clone() });
            board.traces = next.traces;
            board.vias = next.vias;
            self.router = None;
            self.selected = None;
            self.last_report = None;
            self.status = format!("Redo ({} more).", self.redo_stack.len());
        } else {
            self.status = "Nothing to redo.".into();
        }
    }

    fn route(&mut self) {
        self.push_undo();
        let Some(board) = self.board.as_mut() else { return };
        // Keep FIXED copper (pre-existing wiring + locked routes); only drop previously
        // AUTOROUTED traces/vias so re-routing fills in unrouted nets rather than wiping
        // good existing routing. route_board then skips nets that already have copper.
        board.traces.retain(|t| t.fixed == fr_board::FixedState::Fix);
        board.vias.retain(|v| v.fixed == fr_board::FixedState::Fix);
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

    /// Recompute the cached DRC violation list from the current board.
    fn recompute_violations(&mut self) {
        self.violations = self
            .board
            .as_ref()
            .map(fr_engine::drc_violations)
            .unwrap_or_default();
    }

    /// Info windows: incompletes (unrouted nets), DRC violations, board statistics. Each
    /// list is clickable to recenter the view on the item (set goto_point).
    fn info_windows(&mut self, ctx: &egui::Context) {
        let per_unit = self.board.as_ref().map(|b| b.resolution.per_unit as f64).unwrap_or(1.0);

        // Incompletes: unrouted nets and their ratsnest airlines.
        if self.show_incompletes_win {
            let mut open = self.show_incompletes_win;
            let mut goto: Option<Point> = None;
            egui::Window::new("Incompletes (unrouted nets)")
                .open(&mut open)
                .resizable(true)
                .default_size([420.0, 420.0])
                .show(ctx, |ui| {
                    if let (Some(board), Some(rep)) = (&self.board, &self.last_report) {
                        ui.label(format!("{} unrouted of {} nets", rep.unrouted_nets.len(), rep.nets_total));
                        ui.separator();
                        egui::ScrollArea::vertical().show(ui, |ui| {
                            for &nid in &rep.unrouted_nets {
                                let name = board.nets.get(nid).map(|n| n.name.as_str()).unwrap_or("?");
                                let airlines = net_ratsnest(board, nid);
                                let label = format!("{name}  ({} airline(s))", airlines.len());
                                if ui.selectable_label(false, label).clicked() {
                                    if let Some((a, _b)) = airlines.first() {
                                        goto = Some(*a);
                                    }
                                }
                            }
                        });
                    } else {
                        ui.label("Route the board first (▶ Route) to see incompletes.");
                    }
                });
            if goto.is_some() {
                self.goto_point = goto;
            }
            self.show_incompletes_win = open;
        }

        // DRC violations list.
        if self.show_violations_win {
            let mut open = self.show_violations_win;
            let mut goto: Option<Point> = None;
            egui::Window::new(format!("DRC violations ({})", self.violations.len()))
                .open(&mut open)
                .resizable(true)
                .default_size([460.0, 420.0])
                .show(ctx, |ui| {
                    if self.violations.is_empty() {
                        ui.label("No DRC violations. 🎉");
                    } else {
                        egui::ScrollArea::vertical().show(ui, |ui| {
                            for (i, v) in self.violations.iter().enumerate() {
                                let label = format!(
                                    "{}. {} (layer {}) @ ({:.1}, {:.1}) mil",
                                    i + 1, v.kind, v.layer,
                                    v.location.x as f64 / per_unit, v.location.y as f64 / per_unit
                                );
                                if ui.selectable_label(false, label).clicked() {
                                    goto = Some(v.location);
                                }
                            }
                        });
                    }
                });
            if goto.is_some() {
                self.goto_point = goto;
                self.show_violations = true;
            }
            self.show_violations_win = open;
        }

        // Components browser: click to recenter on the component.
        if self.show_components_win {
            let mut open = self.show_components_win;
            let mut goto: Option<Point> = None;
            egui::Window::new("Components")
                .open(&mut open)
                .resizable(true)
                .default_size([320.0, 460.0])
                .show(ctx, |ui| {
                    if let Some(board) = &self.board {
                        ui.add(egui::TextEdit::singleline(&mut self.comp_filter).hint_text("filter…").desired_width(220.0));
                        ui.separator();
                        let filt = self.comp_filter.to_ascii_lowercase();
                        egui::ScrollArea::vertical().show(ui, |ui| {
                            for c in &board.components {
                                if !filt.is_empty() && !c.name.to_ascii_lowercase().contains(&filt) {
                                    continue;
                                }
                                let label = format!("{}  [{}]", c.name, c.image);
                                if ui.selectable_label(false, label).clicked() {
                                    goto = Some(c.location);
                                }
                            }
                        });
                    }
                });
            if goto.is_some() {
                self.goto_point = goto;
            }
            self.show_components_win = open;
        }

        // Nets list: click to highlight the whole net.
        if self.show_nets_win {
            let mut open = self.show_nets_win;
            let mut hl: Option<usize> = None;
            egui::Window::new("Nets")
                .open(&mut open)
                .resizable(true)
                .default_size([300.0, 460.0])
                .show(ctx, |ui| {
                    if let Some(board) = &self.board {
                        ui.add(egui::TextEdit::singleline(&mut self.net_filter).hint_text("filter…").desired_width(220.0));
                        ui.separator();
                        let filt = self.net_filter.to_ascii_lowercase();
                        egui::ScrollArea::vertical().show(ui, |ui| {
                            for (id, net) in board.nets.iter() {
                                if !filt.is_empty() && !net.name.to_ascii_lowercase().contains(&filt) {
                                    continue;
                                }
                                let selected = self.highlight_net == Some(id);
                                if ui.selectable_label(selected, &net.name).clicked() {
                                    hl = Some(id);
                                }
                            }
                        });
                    }
                });
            if let Some(id) = hl {
                self.highlight_net = Some(id);
            }
            self.show_nets_win = open;
        }

        // Net classes viewer/editor: per-class trace width + clearance (in mil), and the
        // member nets. Editing width/clearance updates the class rule (applied by the
        // router via net_class_overrides). A class click highlights its first net.
        if self.show_classes_win {
            let mut open = self.show_classes_win;
            let mut hl_name: Option<String> = None;
            let per_unit = self.board.as_ref().map(|b| b.resolution.per_unit as f64).unwrap_or(10000.0);
            let global_w = self.board.as_ref().map(|b| b.rules.default_width as f64 / per_unit).unwrap_or(0.0);
            let global_c = self.board.as_ref().map(|b| b.rules.default_clearance as f64 / per_unit).unwrap_or(0.0);
            egui::Window::new("Net classes")
                .open(&mut open)
                .resizable(true)
                .default_size([520.0, 460.0])
                .show(ctx, |ui| {
                    let Some(board) = self.board.as_mut() else { return };
                    if board.net_classes.is_empty() {
                        ui.label("This board defines no net classes.");
                        return;
                    }
                    ui.label(format!("Global rule: width {global_w:.1} mil, clearance {global_c:.1} mil. Per-class values below override it (blank = global)."));
                    ui.separator();
                    egui::ScrollArea::vertical().show(ui, |ui| {
                        egui::Grid::new("netclass_grid").striped(true).num_columns(4).show(ui, |ui| {
                            ui.strong("Class");
                            ui.strong("Nets");
                            ui.strong("Width (mil)");
                            ui.strong("Clearance (mil)");
                            ui.end_row();
                            for class in &mut board.net_classes {
                                if ui.selectable_label(false, &class.name).clicked() {
                                    hl_name = class.nets.first().cloned();
                                }
                                ui.label(format!("{}", class.nets.len()));
                                // width: edit in mil; store back in board units. 0 = global.
                                let mut w_mil = class.width.map(|w| w as f64 / per_unit).unwrap_or(0.0);
                                if ui.add(egui::DragValue::new(&mut w_mil).speed(0.5).range(0.0..=200.0)).changed() {
                                    class.width = if w_mil > 0.0 { Some((w_mil * per_unit) as i64) } else { None };
                                }
                                let mut c_mil = class.clearance.map(|c| c as f64 / per_unit).unwrap_or(0.0);
                                if ui.add(egui::DragValue::new(&mut c_mil).speed(0.5).range(0.0..=200.0)).changed() {
                                    class.clearance = if c_mil > 0.0 { Some((c_mil * per_unit) as i64) } else { None };
                                }
                                ui.end_row();
                            }
                        });
                    });
                });
            // highlight the class's first net by name.
            if let Some(net_name) = hl_name {
                if let Some(board) = self.board.as_ref() {
                    self.highlight_net = board.nets.index_of(&net_name);
                }
            }
            self.show_classes_win = open;
        }

        // Help / keyboard shortcuts.
        if self.show_help_win {
            let mut open = self.show_help_win;
            egui::Window::new("Help — controls & shortcuts")
                .open(&mut open)
                .resizable(true)
                .default_size([460.0, 380.0])
                .show(ctx, |ui| {
                    ui.heading("Mouse");
                    ui.monospace("drag           pan");
                    ui.monospace("wheel          zoom");
                    ui.monospace("ctrl/shift+wheel  cycle active layer");
                    ui.monospace("click          select item (highlights its net)");
                    ui.separator();
                    ui.heading("Keyboard");
                    ui.monospace("↑ / ↓  or  [ / ]   cycle active layer");
                    ui.monospace("Ctrl+Z / Ctrl+Y    undo / redo");
                    ui.monospace("Esc                exit manual-route mode");
                    ui.separator();
                    ui.heading("Manual routing");
                    ui.monospace("✏ Manual           toggle route mode");
                    ui.monospace("click a pad        start a route on its net");
                    ui.monospace("click              place a segment");
                    ui.monospace("click dest. pad    complete the connection");
                    ui.monospace("right-click        cancel current route");
                    ui.label("Snap angle, vias, shove, and active layer are in the side panel.");
                });
            self.show_help_win = open;
        }

        // Board statistics.
        if self.show_stats_win {
            let mut open = self.show_stats_win;
            egui::Window::new("Board statistics")
                .open(&mut open)
                .resizable(true)
                .show(ctx, |ui| {
                    if let Some(board) = &self.board {
                        ui.monospace(format!("name:        {}", board.name));
                        ui.monospace(format!("layers:      {}", board.layer_count()));
                        ui.monospace(format!("nets:        {}", board.nets.len()));
                        ui.monospace(format!("components:  {}", board.components.len()));
                        ui.monospace(format!("pins:        {}", board.pins.len()));
                        ui.monospace(format!("padstacks:   {}", board.padstacks.len()));
                        ui.monospace(format!("keepouts:    {}", board.keepouts.len()));
                        ui.monospace(format!("traces:      {}", board.traces.len()));
                        ui.monospace(format!("vias:        {}", board.vias.len()));
                        if let Some(rep) = &self.last_report {
                            ui.separator();
                            ui.monospace(format!("routed:      {}/{} nets", rep.nets_completed, rep.nets_total));
                            ui.monospace(format!("incomplete:  {}", rep.unrouted_nets.len()));
                        }
                    } else {
                        ui.label("No board loaded.");
                    }
                });
            self.show_stats_win = open;
        }
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
    /// one — anchored on the clicked pad/trace/via if one is under the cursor (adopting ITS
    /// net and layer), else at the raw point on the active layer. Otherwise commit a
    /// segment to `p`. Starting on a pad also highlights that net.
    fn route_click(&mut self, p: Point) {
        self.ensure_router();
        if self.router.as_ref().map(|r| !r.has_start()).unwrap_or(false) {
            // begin: snap to a clicked item to pick the start point, net, and layer.
            let tol = 6.0 / self.view.as_ref().map(|v| v.scale).unwrap_or(1.0).max(1e-9);
            let (start, layer, net) = self.route_anchor(p, tol);
            self.highlight_net = Some(net as usize);
            if let Some(router) = self.router.as_mut() {
                router.begin(start, layer, net);
            }
            self.active_layer = layer;
            self.status = format!(
                "Manual route started (net {net}, layer {layer}). Click to place; right-click/Esc to finish."
            );
            return;
        }
        // Commit a segment to the clicked point. Snap the target to a nearby pad/via/trace
        // (its exact point + layer) so clicking a destination PAD actually completes the
        // connection there, rather than missing by a few units / committing on the wrong
        // layer. A target on the same net that's an endpoint finishes the connection.
        let tol = 6.0 / self.view.as_ref().map(|v| v.scale).unwrap_or(1.0).max(1e-9);
        let (target, target_layer, _tnet) = self.route_anchor(p, tol);
        self.push_undo(); // board is about to change
        let Some(router) = self.router.as_mut() else { return };
        let Some(board) = self.board.as_mut() else { return };
        let committed = if self.shove {
            let out = router.commit_shove(board, target, target_layer);
            self.last_report = None;
            if out.committed {
                self.status = format!("Placed segment (shove: {} rerouted, {} dropped).", out.rerouted, out.dropped);
            } else {
                self.status = "No route even with shove (try another path or layer).".into();
            }
            out.committed
        } else if router.commit(board, target, target_layer) {
            self.last_report = None;
            self.status = "Placed a manual route segment.".into();
            true
        } else {
            self.status = "No clear route to that point (enable Shove, or try another path/layer).".into();
            false
        };
        // if we committed onto a pad of this net, the connection is done — finish the route
        // so the next click can start a new one. The router anchor advances to `target`.
        if committed {
            self.active_layer = target_layer;
            if self.snapped_to_pad(target, tol) {
                if let Some(r) = self.router.as_mut() {
                    r.cancel();
                }
                self.status = "Connected to pad — route complete.".into();
            }
        }
    }

    /// True if `p` is within `tol` of a pad center (used to decide a route reached a pad).
    fn snapped_to_pad(&self, p: Point, tol: f64) -> bool {
        let Some(board) = self.board.as_ref() else { return false };
        matches!(
            crate::picking::pick_at(board, p, tol, &self.layer_visible),
            Some(crate::picking::Pick::Pad { .. })
        )
    }

    /// Choose the start anchor for a manual route at click point `p`: if a pad/via/trace is
    /// within `tol` board units, snap to it and adopt its net + layer; otherwise use `p` on
    /// the active layer with net 0.
    fn route_anchor(&self, p: Point, tol: f64) -> (Point, usize, u32) {
        let Some(board) = self.board.as_ref() else { return (p, self.active_layer, 0) };
        if let Some(pick) = crate::picking::pick_at(board, p, tol, &self.layer_visible) {
            match pick {
                crate::picking::Pick::Pad { pin_index } => {
                    if let Some(pin) = board.pins.get(pin_index) {
                        // Start on the ACTIVE layer if the pad's copper covers it (e.g. a
                        // through-hole pad spans all layers — starting on the bottom layer
                        // the user is viewing avoids an immediate jump to layer 0 and a
                        // spurious via). Otherwise start on the pad's first copper layer.
                        let layer = board
                            .padstacks
                            .get(pin.padstack)
                            .map(|ps| {
                                let lo = ps.from_layer().unwrap_or(self.active_layer);
                                let hi = ps.to_layer().unwrap_or(lo);
                                if self.active_layer >= lo && self.active_layer <= hi {
                                    self.active_layer
                                } else {
                                    lo
                                }
                            })
                            .unwrap_or(self.active_layer);
                        return (pin.location, layer, pin.net.unwrap_or(0) as u32);
                    }
                }
                crate::picking::Pick::Via { index } => {
                    if let Some(v) = board.vias.get(index) {
                        return (v.location, self.active_layer, v.net.unwrap_or(0) as u32);
                    }
                }
                crate::picking::Pick::Trace { index } => {
                    if let Some(t) = board.traces.get(index) {
                        let end = *t.corners.last().unwrap_or(&p);
                        return (end, t.layer, t.net.unwrap_or(0) as u32);
                    }
                }
            }
        }
        (p, self.active_layer, 0)
    }

    /// Delete a picked trace or via (pads are footprint copper, not deletable). Snapshots
    /// for undo and invalidates the router/selection.
    fn delete_pick(&mut self, pick: crate::picking::Pick) {
        self.push_undo();
        if let Some(board) = self.board.as_mut() {
            match pick {
                crate::picking::Pick::Trace { index } if index < board.traces.len() => {
                    board.traces.remove(index);
                    self.status = "Deleted trace.".into();
                }
                crate::picking::Pick::Via { index } if index < board.vias.len() => {
                    board.vias.remove(index);
                    self.status = "Deleted via.".into();
                }
                _ => {}
            }
        }
        self.router = None;
        self.last_report = None;
        if self.show_violations {
            self.recompute_violations();
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
        self.push_undo();
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

    /// Per-layer trace color, matching the Java freerouting default scheme: layer 0 (top)
    /// red, layer 1 blue, inner signal layers cycle through 6 greens/oranges/etc.
    fn layer_color(layer: usize) -> Color32 {
        match layer {
            0 => Color32::from_rgb(200, 52, 52),   // top layer: red
            1 => Color32::from_rgb(77, 127, 196),  // second layer: blue
            _ => {
                // inner layers (Java: signal_layer_no % 6)
                const INNER: [Color32; 6] = [
                    Color32::from_rgb(40, 204, 217),  // remainder 0
                    Color32::from_rgb(127, 200, 127), // 1
                    Color32::from_rgb(206, 125, 44),  // 2
                    Color32::from_rgb(79, 203, 203),  // 3
                    Color32::from_rgb(219, 98, 139),  // 4
                    Color32::from_rgb(167, 165, 198), // 5
                ];
                INNER[layer % 6]
            }
        }
    }

    // Java freerouting default palette (boardgraphics OtherColorTableModel / ItemColorTableModel).
    const C_BACKGROUND: Color32 = Color32::from_rgb(0, 16, 35);
    const C_OUTLINE: Color32 = Color32::from_rgb(100, 150, 255);
    const C_PAD: Color32 = Color32::from_rgb(227, 183, 46);     // pins/vias: gold

    /// Color for drawing copper on `layer`, emphasizing the active layer: the active layer
    /// (or all layers when fade is off) gets its full palette color; other layers are
    /// desaturated toward gray and dimmed, so the current layer reads as "on top". Returns
    /// the color and a draw-order hint (false = faded/behind, true = active/front).
    fn layer_style(&self, layer: usize) -> (Color32, bool) {
        let base = Self::layer_color(layer);
        if !self.fade_inactive || layer == self.active_layer {
            return (base, layer == self.active_layer);
        }
        // desaturate toward the channel average, then dim (toward the dark background).
        let avg = ((base.r() as u32 + base.g() as u32 + base.b() as u32) / 3) as u8;
        let mix = |c: u8| -> u8 {
            // 60% toward gray, then scaled to ~45% brightness
            let desat = (c as u32 * 40 + avg as u32 * 60) / 100;
            (desat * 45 / 100) as u8
        };
        (Color32::from_rgb(mix(base.r()), mix(base.g()), mix(base.b())), false)
    }

    fn draw_board(&mut self, ui: &mut egui::Ui) {
        let Some(board) = &self.board else {
            ui.centered_and_justified(|ui| { ui.label("No board loaded."); });
            return;
        };
        let (resp, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let screen = resp.rect;
        // Dark neutral backdrop, clearly distinct from the (greener) board substrate.
        painter.rect_filled(screen, 0.0, Self::C_BACKGROUND);

        if self.view.is_none() || self.refit {
            let bounds = board
                .outline_box()
                .or_else(|| fr_geometry::IntBox::bound(board.pins.iter().map(|p| p.location)))
                .unwrap_or(fr_geometry::IntBox::new(0, 0, 1, 1));
            self.view = Some(ViewTransform::fit(bounds, screen));
            self.refit = false;
        }
        let (scroll, mods) = ui.input(|i| (i.smooth_scroll_delta.y, i.modifiers));
        // Ctrl/Shift + wheel cycles the active layer (plain wheel stays zoom). Capture the
        // intent; apply after the board borrow ends (cycle_layer borrows self mutably).
        let mut layer_wheel: i64 = 0;
        // Pan/zoom mutate the view; scope that borrow tightly so the rest of draw_board can
        // call &self methods (layer_style) while only reading the view via `view_ro`.
        {
            let view = self.view.as_mut().unwrap();
            if resp.dragged() {
                view.pan_pixels(resp.drag_delta());
            }
            if scroll.abs() > 0.0 {
                if mods.ctrl || mods.shift {
                    layer_wheel = if scroll > 0.0 { 1 } else { -1 };
                } else {
                    let anchor = resp.hover_pos().unwrap_or(screen.center());
                    view.zoom_at((scroll as f64 * 0.002).exp(), anchor, screen);
                }
            }
            // recenter on a requested point (from an info-list click)
            if let Some(p) = self.goto_point.take() {
                view.center = p;
            }
        }
        let view_ro = *self.view.as_ref().unwrap();
        // track the cursor position (board units) for the status bar.
        self.cursor_pos = resp.hover_pos().map(|hp| view_ro.to_board(hp, screen));

        // Hit-test under the cursor (within a few-pixel tolerance, converted to board
        // units). Used for the hover tooltip and click selection. A drag is a pan, not a
        // pick, so only treat a non-drag click as a selection.
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
        // Right-click cancels the in-progress route (stays in route mode); Esc exits route
        // mode entirely.
        let route_cancel = self.route_mode && resp.secondary_clicked();
        let route_exit = self.route_mode && esc;
        let new_selection: Option<Option<crate::picking::Pick>> =
            if resp.clicked() && !self.route_mode { Some(hover_pick) } else { None };

        // board outline: filled substrate (concave-safe via ear-clipping) + edge stroke,
        // giving clear board/background contrast. The fill is built as an explicit Mesh
        // (winding-independent) rather than convex_polygon, because to_screen flips Y and
        // egui's convex-polygon fill is winding-sensitive (it can drop reversed triangles
        // on some backends). The Mesh rasterizes regardless of vertex order.
        if board.outline.len() >= 3 {
            // subtle lift over the background so the board area reads, with the Java blue
            // outline. (Java draws the board on the background; we add a faint fill for
            // contrast against off-board.)
            let board_fill = Color32::from_rgb(10, 30, 58);
            let mut mesh = egui::epaint::Mesh::default();
            for tri in crate::padgeom::triangulate(&board.outline) {
                let base = mesh.vertices.len() as u32;
                for &vi in &tri {
                    mesh.colored_vertex(view_ro.to_screen(board.outline[vi], screen), board_fill);
                }
                mesh.add_triangle(base, base + 1, base + 2);
            }
            if !mesh.is_empty() {
                painter.add(egui::Shape::mesh(mesh));
            }
            // bright, bold edge so the board boundary is unmistakable.
            let edge: Vec<Pos2> = board.outline.iter().map(|&p| view_ro.to_screen(p, screen)).collect();
            for i in 0..edge.len() {
                painter.line_segment(
                    [edge[i], edge[(i + 1) % edge.len()]],
                    Stroke::new(2.0, Self::C_OUTLINE),
                );
            }
        }

        // keepout regions: hatched-look translucent fill + dashed outline (Java keepout
        // color, teal-ish), so the user sees where routing is forbidden.
        if self.show_keepouts {
            let ko_fill = Color32::from_rgba_unmultiplied(26, 196, 210, 28);
            let ko_edge = Color32::from_rgba_unmultiplied(26, 196, 210, 160);
            for ko in &board.keepouts {
                if ko.polygon.len() < 3 {
                    continue;
                }
                for tri in crate::padgeom::triangulate(&ko.polygon) {
                    let mut mesh = egui::epaint::Mesh::default();
                    for &vi in &tri {
                        mesh.colored_vertex(view_ro.to_screen(ko.polygon[vi], screen), ko_fill);
                    }
                    mesh.add_triangle(0, 1, 2);
                    painter.add(egui::Shape::mesh(mesh));
                }
                let pts: Vec<Pos2> = ko.polygon.iter().map(|&p| view_ro.to_screen(p, screen)).collect();
                for i in 0..pts.len() {
                    painter.line_segment([pts[i], pts[(i + 1) % pts.len()]], Stroke::new(1.0, ko_edge));
                }
            }
        }

        // pads: real per-layer copper geometry (circle radius / convex polygon), scaled and
        // tinted by the layer the pad sits on (active layer prominent, others faded). Draw
        // faded pads first, then active-layer pads, so the current layer reads as on top.
        // A highlighted net's pads override with a bright accent.
        if self.show_pads {
            let draw_pad = |pin: &fr_board::Pin, painter: &egui::Painter, front_pass: bool| {
                let Some(shape) = crate::padgeom::pin_pad_shape(board, pin) else { return };
                let player = pad_layer(board, pin, self.active_layer);
                let (lcol, is_active) = self.layer_style(player);
                if is_active != front_pass {
                    return; // two-pass: faded behind, active in front
                }
                let hl = self.highlight_net.is_some() && pin.net == self.highlight_net;
                let col = if hl { Self::C_PAD } else { lcol };
                match shape {
                    crate::padgeom::PadDraw::Circle { center, radius } => {
                        let c = view_ro.to_screen(center, screen);
                        let r = ((radius as f64 * view_ro.scale) as f32).max(1.0);
                        painter.circle_filled(c, r, col);
                    }
                    crate::padgeom::PadDraw::Poly(verts) => {
                        // Mesh fill (winding-independent under the Y-flip), fan-triangulated
                        // since pad polygons are convex.
                        let pts: Vec<Pos2> = verts.iter().map(|&p| view_ro.to_screen(p, screen)).collect();
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
            };
            for pin in &board.pins {
                draw_pad(pin, &painter, false); // faded (behind)
            }
            for pin in &board.pins {
                draw_pad(pin, &painter, true); // active layer (front)
            }
        }

        // clearance boundaries: the keepout halo (copper + design clearance) around
        // obstacles, so the user sees where routing can't go. Shown when toggled on or in
        // route mode (where it's most useful); a faint magenta outline.
        if self.show_clearance || self.route_mode {
            let clr = board.rules.default_clearance as f64;
            let halo = Color32::from_rgba_unmultiplied(230, 120, 230, 90);
            for pin in &board.pins {
                if let Some(shape) = crate::padgeom::pin_pad_shape(board, pin) {
                    match shape {
                        crate::padgeom::PadDraw::Circle { center, radius } => {
                            let r = ((radius as f64 + clr) * view_ro.scale) as f32;
                            painter.circle_stroke(view_ro.to_screen(center, screen), r.max(1.0),
                                Stroke::new(1.0, halo));
                        }
                        crate::padgeom::PadDraw::Poly(verts) => {
                            // approximate the inflated polygon by an outline offset: draw the
                            // polygon edges plus a ring at the centroid radius+clr.
                            let pts: Vec<Pos2> = verts.iter().map(|&p| view_ro.to_screen(p, screen)).collect();
                            for i in 0..pts.len() {
                                painter.line_segment([pts[i], pts[(i + 1) % pts.len()]], Stroke::new(1.0, halo));
                            }
                        }
                    }
                }
            }
            // trace clearance: a wider faint stroke around each visible trace.
            for t in &board.traces {
                if t.layer >= self.layer_visible.len() || !self.layer_visible[t.layer] {
                    continue;
                }
                let w = (((t.width as f64 + 2.0 * clr) * view_ro.scale).max(1.0) as f32).min(40.0);
                for seg in t.corners.windows(2) {
                    painter.line_segment(
                        [view_ro.to_screen(seg[0], screen), view_ro.to_screen(seg[1], screen)],
                        Stroke::new(w, Color32::from_rgba_unmultiplied(230, 120, 230, 40)),
                    );
                }
            }
        }

        // ratsnest of unrouted nets (thin gray lines between unconnected pins)
        if self.show_ratsnest {
            if let Some(rep) = &self.last_report {
                for &nid in &rep.unrouted_nets {
                    for (a, b) in net_ratsnest(board, nid) {
                        painter.line_segment(
                            [view_ro.to_screen(a, screen), view_ro.to_screen(b, screen)],
                            Stroke::new(0.5, Color32::from_rgb(90, 90, 70)),
                        );
                    }
                }
            }
        }

        // traces: per-layer color with active-layer emphasis. Two passes (faded then
        // active) so current-layer traces draw on top of the others.
        for front_pass in [false, true] {
            for t in &board.traces {
                if t.layer >= self.layer_visible.len() || !self.layer_visible[t.layer] {
                    continue;
                }
                let (lcol, is_active) = self.layer_style(t.layer);
                if is_active != front_pass {
                    continue;
                }
                let mut col = lcol;
                if self.highlight_net.is_some() && t.net != self.highlight_net {
                    col = col.gamma_multiply(0.25);
                }
                let w = (((t.width as f64) * view_ro.scale).max(1.0) as f32).min(6.0);
                for seg in t.corners.windows(2) {
                    painter.line_segment(
                        [view_ro.to_screen(seg[0], screen), view_ro.to_screen(seg[1], screen)],
                        Stroke::new(w, col),
                    );
                }
            }
        }

        // vias: span layers, so always drawn prominent (white ring), dimmed only when a
        // different net is highlighted.
        for v in &board.vias {
            let c = view_ro.to_screen(v.location, screen);
            let dim = self.highlight_net.is_some() && v.net != self.highlight_net;
            let col = if dim { Color32::from_gray(120) } else { Self::C_PAD };
            painter.circle_stroke(c, 3.0, Stroke::new(1.5, col));
        }

        // DRC violation markers (magenta crosses) when enabled.
        if self.show_violations {
            for v in &self.violations {
                let c = view_ro.to_screen(v.location, screen);
                let m = Color32::from_rgb(255, 0, 255);
                painter.line_segment([c + egui::vec2(-5.0, -5.0), c + egui::vec2(5.0, 5.0)], Stroke::new(1.5, m));
                painter.line_segment([c + egui::vec2(-5.0, 5.0), c + egui::vec2(5.0, -5.0)], Stroke::new(1.5, m));
            }
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

        // Manual-route guidance: highlight the active net's pads and show a direction
        // indicator from the route start to the NEAREST same-net pad (the likely next
        // target), so the user knows where to route. Active net = highlighted net (or 0).
        if self.route_mode {
            let active_net = self.highlight_net.unwrap_or(0);
            // ring every pad of the active net in a bright accent.
            for pin in &board.pins {
                if pin.net == Some(active_net) {
                    painter.circle_stroke(
                        view_ro.to_screen(pin.location, screen), 6.0,
                        Stroke::new(1.5, Color32::from_rgb(255, 230, 120)),
                    );
                }
            }
            // direction to the nearest active-net pad from the route start (or cursor).
            if let Some(router) = self.router.as_ref() {
                let from = router.start_point().map(|(p, _)| p).or(cursor_board);
                if let Some(from) = from {
                    let nearest = board
                        .pins
                        .iter()
                        .filter(|p| p.net == Some(active_net) && p.location != from)
                        .min_by_key(|p| from.distance_square(p.location));
                    if let Some(t) = nearest {
                        let a = view_ro.to_screen(from, screen);
                        let b = view_ro.to_screen(t.location, screen);
                        // dashed guide line + arrowhead toward the target.
                        painter.add(egui::Shape::dashed_line(
                            &[a, b],
                            Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 230, 120, 140)),
                            6.0, 4.0,
                        ));
                        draw_arrowhead(&painter, a, b, Color32::from_rgb(255, 230, 120));
                    }
                }
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

        // hover tooltip with item info (suppressed in route mode to avoid clutter). Give it
        // a wide min width and disable wrapping so each info line stays on one line.
        if !self.route_mode {
            if let Some(hp) = hover_pick {
                let text = Self::pick_info(board, hp);
                egui::show_tooltip_at_pointer(ui.ctx(), ui.layer_id(), egui::Id::new("pick_tip"), |ui| {
                    ui.set_min_width(340.0);
                    ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Extend);
                    ui.label(egui::RichText::new(text).monospace());
                });
            }
        }

        // Selecting an item highlights its whole net (all pads/vias/traces on that net).
        if let Some(sel) = new_selection {
            self.selected = sel;
            self.highlight_net = sel.and_then(|p| pick_net(board, p));
        }
        // apply captured route-mode intents (board borrow has ended)
        if route_exit {
            self.route_finish();
            self.route_mode = false;
            self.status = "Exited manual route mode.".into();
        } else if route_cancel {
            self.route_finish();
        } else if let Some(p) = route_click_at {
            self.route_click(p);
        }
        if layer_wheel != 0 {
            self.cycle_layer(layer_wheel);
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

impl App {
    /// Number of layers in the loaded board (>=1), or 1 if none.
    fn layer_count(&self) -> usize {
        self.board.as_ref().map(|b| b.layer_count().max(1)).unwrap_or(1)
    }

    /// Cycle the active layer by `delta` (wrapping), keeping the manual router (which uses
    /// the active layer) in sync.
    fn cycle_layer(&mut self, delta: i64) {
        let n = self.layer_count() as i64;
        if n <= 1 {
            return;
        }
        self.active_layer = (((self.active_layer as i64 + delta) % n + n) % n) as usize;
        self.status = self
            .board
            .as_ref()
            .and_then(|b| b.layers.layers().get(self.active_layer))
            .map(|l| format!("Active layer: {} ({})", self.active_layer, l.name))
            .unwrap_or_else(|| format!("Active layer: {}", self.active_layer));
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let has_board = self.board.is_some();

        // Keyboard: ↑/↓ (and [ / ] as aliases) step the active layer; Ctrl+Z / Ctrl+Y
        // (or Ctrl+Shift+Z) undo/redo. Mouse wheel is reserved for zoom; Ctrl/Shift+wheel
        // cycles layers (handled in the canvas). Ignore while a text field has focus.
        if has_board && !ctx.wants_keyboard_input() {
            let (mut dn, mut up, mut do_undo, mut do_redo) = (false, false, false, false);
            ctx.input(|i| {
                dn = i.key_pressed(egui::Key::OpenBracket) || i.key_pressed(egui::Key::ArrowDown);
                up = i.key_pressed(egui::Key::CloseBracket) || i.key_pressed(egui::Key::ArrowUp);
                let ctrl = i.modifiers.ctrl || i.modifiers.command;
                if ctrl && i.key_pressed(egui::Key::Z) {
                    if i.modifiers.shift { do_redo = true; } else { do_undo = true; }
                }
                if ctrl && i.key_pressed(egui::Key::Y) {
                    do_redo = true;
                }
            });
            if dn {
                self.cycle_layer(-1);
            }
            if up {
                self.cycle_layer(1);
            }
            if do_undo {
                self.undo();
            }
            if do_redo {
                self.redo();
            }
        }

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
                // Manual-route mode toggle (primary action). Highlighted when active.
                let route_btn = egui::Button::new("✏ Manual")
                    .selected(self.route_mode);
                if ui.add_enabled(has_board, route_btn).clicked() {
                    self.route_mode = !self.route_mode;
                    if self.route_mode {
                        self.ensure_router();
                        self.status = "Manual route mode: click a pad to start, click to place, Esc to exit.".into();
                    } else {
                        self.route_finish();
                    }
                }
                if ui.add_enabled(has_board, egui::Button::new("Clear")).clicked() {
                    self.clear_routes();
                }
                if ui.add_enabled(has_board, egui::Button::new("Fit")).clicked() {
                    self.refit = true;
                }
                ui.separator();
                if ui.add_enabled(!self.undo_stack.is_empty(), egui::Button::new("↶ Undo")).clicked() {
                    self.undo();
                }
                if ui.add_enabled(!self.redo_stack.is_empty(), egui::Button::new("↷ Redo")).clicked() {
                    self.redo();
                }
                ui.separator();
                if ui.add_enabled(has_board, egui::Button::new("Export RTE")).clicked() {
                    self.export("rte");
                }
                if ui.add_enabled(has_board, egui::Button::new("Export SES")).clicked() {
                    self.export("ses");
                }
                ui.separator();
                // display toggles + info windows (Java toolbar parity).
                if ui.add_enabled(has_board, egui::Button::new("Ratsnest").selected(self.show_ratsnest)).clicked() {
                    self.show_ratsnest = !self.show_ratsnest;
                }
                if ui.add_enabled(has_board, egui::Button::new("Violations").selected(self.show_violations)).clicked() {
                    self.show_violations = !self.show_violations;
                    if self.show_violations {
                        self.recompute_violations();
                    }
                }
                if ui.add_enabled(has_board, egui::Button::new("Incompletes…")).clicked() {
                    self.show_incompletes_win = !self.show_incompletes_win;
                }
                if ui.add_enabled(has_board, egui::Button::new("DRC…")).clicked() {
                    self.show_violations_win = !self.show_violations_win;
                    self.recompute_violations();
                }
                if ui.add_enabled(has_board, egui::Button::new("Components…")).clicked() {
                    self.show_components_win = !self.show_components_win;
                }
                if ui.add_enabled(has_board, egui::Button::new("Nets…")).clicked() {
                    self.show_nets_win = !self.show_nets_win;
                }
                if ui.add_enabled(has_board, egui::Button::new("Classes…")).clicked() {
                    self.show_classes_win = !self.show_classes_win;
                }
                if ui.add_enabled(has_board, egui::Button::new("Stats…")).clicked() {
                    self.show_stats_win = !self.show_stats_win;
                }
                if ui.button("?").on_hover_text("Keyboard shortcuts & help").clicked() {
                    self.show_help_win = !self.show_help_win;
                }
                if !self.warnings.is_empty() {
                    ui.separator();
                    // warning button with a count badge; amber so it stands out.
                    let label = format!("⚠ Warnings ({})", self.warnings.len());
                    let btn = egui::Button::new(egui::RichText::new(label).color(Color32::from_rgb(240, 200, 80)));
                    if ui.add(btn).clicked() {
                        self.show_warnings = !self.show_warnings;
                    }
                }
            });
        });

        self.info_windows(ctx);

        // Warnings window: the DSN-load warnings (missing images, shapeless padstacks,
        // unparsed shapes, …). Toggled from the toolbar.
        if self.show_warnings {
            let mut open = self.show_warnings;
            egui::Window::new(format!("Load warnings ({})", self.warnings.len()))
                .open(&mut open)
                .resizable(true)
                .default_size([560.0, 360.0])
                .show(ctx, |ui| {
                    if self.warnings.is_empty() {
                        ui.label("No warnings.");
                    } else {
                        egui::ScrollArea::vertical().show(ui, |ui| {
                            for (i, w) in self.warnings.iter().enumerate() {
                                ui.label(format!("{}. {}", i + 1, w));
                            }
                        });
                    }
                });
            self.show_warnings = open;
        }

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
                    ui.checkbox(&mut self.fade_inactive, "Fade inactive layers");
                    ui.checkbox(&mut self.show_clearance, "Clearance halos (always)");
                    ui.checkbox(&mut self.show_keepouts, "Keepouts");
                    ui.checkbox(&mut self.show_violations, "DRC violation markers");
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
                let mut do_delete: Option<crate::picking::Pick> = None;
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
                    let is_pad = matches!(sel, crate::picking::Pick::Pad { .. });
                    ui.collapsing("Selection", |ui| {
                        ui.label(info);
                        ui.horizontal(|ui| {
                            if net.is_some() && ui.button("Highlight net").clicked() {
                                do_highlight_net = net;
                            }
                            // a pad is part of the footprint, not deletable; traces/vias are.
                            if !is_pad && ui.button("🗑 Delete").clicked() {
                                do_delete = Some(sel);
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
                if let Some(sel) = do_delete {
                    self.delete_pick(sel);
                    do_clear_selection = true;
                }
                if do_clear_selection {
                    self.selected = None;
                }

                ui.collapsing("Layers", |ui| {
                    if let Some(board) = &self.board {
                        ui.label("● = active (radio); ☑ = visible. [ ] or ←/→ to cycle.");
                        for (i, layer) in board.layers.layers().iter().enumerate() {
                            if i < self.layer_visible.len() {
                                ui.horizontal(|ui| {
                                    // active-layer radio
                                    if ui.radio(self.active_layer == i, "").clicked() {
                                        self.active_layer = i;
                                    }
                                    ui.checkbox(&mut self.layer_visible[i], "");
                                    let (rect, _) = ui.allocate_exact_size(egui::vec2(12.0, 12.0), Sense::hover());
                                    ui.painter().rect_filled(rect, 2.0, App::layer_color(i));
                                    let name = if self.active_layer == i {
                                        egui::RichText::new(&layer.name).strong()
                                    } else {
                                        egui::RichText::new(&layer.name)
                                    };
                                    ui.label(name);
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
            ui.horizontal(|ui| {
                ui.label(&self.status);
                // right-aligned: active layer + cursor coordinates (in mil), like Java.
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if let (Some(board), Some(p)) = (&self.board, self.cursor_pos) {
                        let per = board.resolution.per_unit as f64;
                        ui.monospace(format!("({:.1}, {:.1}) mil", p.x as f64 / per, p.y as f64 / per));
                        ui.separator();
                    }
                    if let Some(board) = &self.board {
                        let lname = board.layers.layers().get(self.active_layer).map(|l| l.name.as_str()).unwrap_or("?");
                        ui.monospace(format!("layer: {} ({})", self.active_layer, lname));
                    }
                });
            });
        });

        egui::CentralPanel::default().show(ctx, |ui| {
            self.draw_board(ui);
        });

        self.file_browser(ctx);
    }
}

/// The net of a picked item (pad/via/trace), if any.
fn pick_net(board: &Board, pick: crate::picking::Pick) -> Option<usize> {
    match pick {
        crate::picking::Pick::Pad { pin_index } => board.pins.get(pin_index).and_then(|p| p.net),
        crate::picking::Pick::Via { index } => board.vias.get(index).and_then(|v| v.net),
        crate::picking::Pick::Trace { index } => board.traces.get(index).and_then(|t| t.net),
    }
}

/// Draw a small arrowhead at `b` pointing along a->b.
fn draw_arrowhead(painter: &egui::Painter, a: Pos2, b: Pos2, color: Color32) {
    let dir = b - a;
    let len = dir.length();
    if len < 1.0 {
        return;
    }
    let u = dir / len;
    let perp = egui::vec2(-u.y, u.x);
    let size = 8.0;
    let tip = b;
    let left = b - u * size + perp * (size * 0.5);
    let right = b - u * size - perp * (size * 0.5);
    painter.add(egui::Shape::convex_polygon(vec![tip, left, right], color, Stroke::NONE));
}

/// The representative layer to color a pad by: the active layer if the pin's padstack
/// carries copper there (so a pad on the current layer reads as active), otherwise the
/// padstack's top copper layer. Falls back to 0.
fn pad_layer(board: &Board, pin: &fr_board::Pin, active: usize) -> usize {
    let Some(ps) = board.padstacks.get(pin.padstack) else { return 0 };
    let lo = ps.from_layer().unwrap_or(0);
    let hi = ps.to_layer().unwrap_or(lo);
    if active >= lo && active <= hi {
        active
    } else {
        lo
    }
}

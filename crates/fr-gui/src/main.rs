//! freerouting-rs GUI (egui/eframe). Board viewer + autorouter front-end.
//!
//!   freerouting-rs-gui [BOARD.dsn]
//!
//! Opens the optional board on launch. Runs under a normal desktop or, for CI
//! verification, under xvfb (see harness / MILESTONES Phase 7).

mod app;
mod view;

use app::App;

fn main() -> eframe::Result<()> {
    let arg = std::env::args().nth(1);
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 800.0])
            .with_title("freerouting-rs"),
        ..Default::default()
    };
    eframe::run_native(
        "freerouting-rs",
        options,
        Box::new(move |_cc| {
            let mut a = App::default();
            if let Some(path) = arg {
                a.load_path(path.into());
            }
            Ok(Box::new(a))
        }),
    )
}

//! freerouting-rs GUI (egui/eframe). Board viewer + autorouter front-end.
//!
//!   freerouting-rs-gui [BOARD.dsn]
//!
//! Opens the optional board on launch. Runs under a normal desktop or, for CI
//! verification, under xvfb (see harness / MILESTONES Phase 7).

mod app;
mod render;
mod view;

use app::App;

fn main() -> eframe::Result<()> {
    let args: Vec<String> = std::env::args().collect();

    // Headless render mode: `freerouting-rs-gui --render BOARD.dsn OUT.ppm [W H]`.
    // Routes the board and rasterizes it to a PPM with the software renderer - no
    // window/GL needed, so the render path is verifiable in CI / headless.
    if args.get(1).map(|s| s == "--render").unwrap_or(false) {
        let dsn = args.get(2).expect("--render needs a DSN path");
        let out = args.get(3).expect("--render needs an output .ppm path");
        let w: u32 = args.get(4).and_then(|s| s.parse().ok()).unwrap_or(1200);
        let h: u32 = args.get(5).and_then(|s| s.parse().ok()).unwrap_or(800);
        let src = std::fs::read_to_string(dsn).expect("read dsn");
        let (mut board, _w) = fr_dsn::read_board(&src);
        let _ = fr_engine::route_board(&mut board, &fr_engine::RouteOptions::default());
        let img = render::render_board(&board, w, h);
        std::fs::write(out, img.to_ppm()).expect("write ppm");
        eprintln!("rendered {} ({}x{}) with {} traces, {} vias", out, w, h, board.traces.len(), board.vias.len());
        std::process::exit(0);
    }

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

//! freerouting-rs unified entry point.
//!
//! By default this launches the interactive GUI (egui/eframe board viewer + autorouter
//! front-end). With `--headless` it batch-routes a board and writes an Altium-importable
//! `.rte`/`.ses` without opening a window (no GL/display needed) — the same routing the
//! GUI's Route button runs. `--render` rasterizes a routed board to a PPM image (also
//! headless), used for CI verification of the render path.
//!
//! Examples:
//!   freerouting-rs-gui                      # GUI, empty
//!   freerouting-rs-gui board.dsn            # GUI, board open on launch
//!   freerouting-rs-gui --headless board.dsn -o out.rte --max-time 30
//!   freerouting-rs-gui --render board.dsn out.ppm 1400 1000

mod app;
mod padgeom;
mod picking;
mod render;
mod view;

use std::path::PathBuf;
use std::process::ExitCode;

use app::App;
use clap::Parser;

#[derive(Parser, Debug)]
#[command(name = "freerouting-rs", about = "freerouting-rs: PCB autorouter (GUI by default; --headless to batch-route)")]
struct Cli {
    /// Board to open (GUI) or route (--headless). Optional in GUI mode.
    input: Option<PathBuf>,

    /// Batch-route the board and write the result without opening a window.
    #[arg(long)]
    headless: bool,

    /// Output file for --headless (.rte or .ses; extension selects the format).
    /// Defaults to the input path with a .rte extension.
    #[arg(short, long)]
    output: Option<PathBuf>,

    /// Wall-clock routing budget in seconds (0 = until done / pass cap). --headless only.
    #[arg(long, default_value_t = 0)]
    max_time: u64,

    /// Worker threads (0 = auto). --headless only.
    #[arg(long, default_value_t = 0)]
    threads: usize,

    /// Deterministic seed. --headless only.
    #[arg(long, default_value_t = 1)]
    seed: u64,

    /// Rasterize the routed board to a PPM image instead of routing to RTE/SES.
    /// Value is the output .ppm path; size is set with --width/--height. Implies headless.
    #[arg(long, value_name = "OUT.ppm")]
    render: Option<PathBuf>,

    /// Render image width (with --render).
    #[arg(long, default_value_t = 1200)]
    width: u32,

    /// Render image height (with --render).
    #[arg(long, default_value_t = 800)]
    height: u32,
}

fn main() -> ExitCode {
    let cli = Cli::parse();

    if let Some(out) = cli.render.clone() {
        return match run_render(&cli, &out) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        };
    }

    if cli.headless {
        return match run_headless(&cli) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        };
    }

    match run_gui(cli.input) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("error: {e}");
            ExitCode::FAILURE
        }
    }
}

/// Headless batch route: DSN -> RTE/SES, no window. Mirrors the engine the GUI's Route
/// button uses, with the same console summary as the CLI.
fn run_headless(cli: &Cli) -> anyhow::Result<()> {
    let input = cli
        .input
        .clone()
        .ok_or_else(|| anyhow::anyhow!("--headless needs an input DSN path"))?;
    let output = cli
        .output
        .clone()
        .unwrap_or_else(|| input.with_extension("rte"));

    let src = std::fs::read_to_string(&input)?;
    let (mut board, warnings) = fr_dsn::read_board(&src);
    eprintln!(
        "loaded {}: {} layers, {} nets, {} components, {} pre-existing wires ({} warnings)",
        board.name,
        board.layer_count(),
        board.nets.len(),
        board.components.len(),
        board.traces.len(),
        warnings.len()
    );
    // autoroute from a clean slate (drop the source design's pre-existing wiring).
    board.traces.clear();
    board.vias.clear();

    let opts = fr_engine::RouteOptions {
        max_time_secs: cli.max_time,
        threads: cli.threads,
        seed: cli.seed,
        width: 0,
        clearance: 0,
        max_layers: 0,
    };
    let report = fr_engine::route_board(&mut board, &opts);
    eprintln!(
        "routed: {}/{} nets, {} traces, {} vias in {} passes",
        report.nets_completed,
        report.nets_total,
        board.traces.len(),
        board.vias.len(),
        report.passes
    );

    let ext = output
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("rte")
        .to_lowercase();
    let out = if ext == "ses" {
        fr_dsn::write_ses(&board)
    } else {
        fr_dsn::write_rte(&board)
    };
    std::fs::write(&output, out)?;
    eprintln!("wrote {}", output.display());
    Ok(())
}

/// Route a board and rasterize it to a PPM with the software renderer (no window/GL).
fn run_render(cli: &Cli, out: &PathBuf) -> anyhow::Result<()> {
    let input = cli
        .input
        .clone()
        .ok_or_else(|| anyhow::anyhow!("--render needs an input DSN path"))?;
    let src = std::fs::read_to_string(&input)?;
    let (mut board, _w) = fr_dsn::read_board(&src);
    // autoroute from a clean slate (drop pre-existing wiring) so the render shows OUR route.
    board.traces.clear();
    board.vias.clear();
    let _ = fr_engine::route_board(&mut board, &fr_engine::RouteOptions::default());
    let img = render::render_board(&board, cli.width, cli.height);
    std::fs::write(out, img.to_ppm())?;
    eprintln!(
        "rendered {} ({}x{}) with {} traces, {} vias",
        out.display(),
        cli.width,
        cli.height,
        board.traces.len(),
        board.vias.len()
    );
    Ok(())
}

/// Launch the interactive GUI, optionally opening `input` on start.
fn run_gui(input: Option<PathBuf>) -> anyhow::Result<()> {
    // Robust wgpu config for headless/WSLg stacks: FIFO present mode is universally
    // supported (the default AutoVsync can request a mode the WSLg surface rejects with
    // "surface isn't supported by this adapter"), low-power prefers the software/lavapipe
    // adapter, and a skip-frame surface-error handler avoids a hard panic on a transient
    // surface loss (e.g. a display hiccup or a second instance grabbing the surface).
    let mut wgpu_options = eframe::egui_wgpu::WgpuConfiguration::default();
    wgpu_options.present_mode = eframe::wgpu::PresentMode::Fifo;
    wgpu_options.power_preference = eframe::wgpu::PowerPreference::LowPower;
    wgpu_options.on_surface_error = std::sync::Arc::new(|err| {
        eprintln!("wgpu surface error (skipping frame): {err:?}");
        eframe::egui_wgpu::SurfaceErrorAction::SkipFrame
    });

    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1200.0, 800.0])
            .with_title("freerouting-rs"),
        wgpu_options,
        ..Default::default()
    };
    eframe::run_native(
        "freerouting-rs",
        options,
        Box::new(move |cc| {
            // modern dark visuals: a touch more spacing + rounding than the egui default.
            use eframe::egui;
            let ctx = &cc.egui_ctx;
            ctx.set_visuals(egui::Visuals::dark());
            ctx.style_mut(|s| {
                s.spacing.item_spacing = egui::vec2(8.0, 6.0);
                s.spacing.button_padding = egui::vec2(8.0, 4.0);
                s.visuals.widgets.noninteractive.rounding = egui::Rounding::same(4.0);
                s.visuals.widgets.inactive.rounding = egui::Rounding::same(4.0);
                s.visuals.widgets.hovered.rounding = egui::Rounding::same(4.0);
                s.visuals.widgets.active.rounding = egui::Rounding::same(4.0);
            });
            let mut a = App::default();
            if let Some(path) = input {
                a.load_path(path);
            }
            Ok(Box::new(a))
        }),
    )
    .map_err(|e| anyhow::anyhow!("GUI failed: {e}"))
}

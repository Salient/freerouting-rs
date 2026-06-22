//! freerouting-rs CLI: headless route/convert entry point used by the self-verifying
//! gates and the Altium round-trip harness.
//!
//!   freerouting-rs route IN.dsn -o OUT.rte [--max-time S] [--threads N] [--ses]
//!   freerouting-rs info  IN.dsn
//!
//! `route` reads a Specctra DSN, runs the autorouter, and writes an Altium-importable
//! route (.rte) or session (.ses) file.

use std::fs;
use std::path::PathBuf;
use std::process::ExitCode;

use clap::{Parser, Subcommand};

#[derive(Parser)]
#[command(name = "freerouting-rs", version, about = "Fast multithreaded PCB autorouter")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Route a DSN and write an Altium-importable .rte (or .ses).
    Route {
        /// Input Specctra DSN file.
        input: PathBuf,
        /// Output file (.rte or .ses). Extension selects the format.
        #[arg(short, long)]
        output: PathBuf,
        /// Wall-clock routing budget in seconds (0 = until done / pass cap).
        #[arg(long, default_value_t = 0)]
        max_time: u64,
        /// Worker threads (0 = auto).
        #[arg(long, default_value_t = 0)]
        threads: usize,
        /// Deterministic seed.
        #[arg(long, default_value_t = 1)]
        seed: u64,
    },
    /// Print a summary of a DSN without routing.
    Info {
        input: PathBuf,
    },
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    match cli.cmd {
        Cmd::Info { input } => match run_info(&input) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("error: {e}");
                ExitCode::FAILURE
            }
        },
        Cmd::Route { input, output, max_time, threads, seed } => {
            match run_route(&input, &output, max_time, threads, seed) {
                Ok(()) => ExitCode::SUCCESS,
                Err(e) => {
                    eprintln!("error: {e}");
                    ExitCode::FAILURE
                }
            }
        }
    }
}

fn run_info(input: &PathBuf) -> anyhow::Result<()> {
    let src = fs::read_to_string(input)?;
    let (board, warnings) = fr_dsn::read_board(&src);
    println!("board:       {}", board.name);
    println!("resolution:  {} {}", board.resolution.unit.as_str(), board.resolution.per_unit);
    println!("layers:      {}", board.layer_count());
    println!("nets:        {}", board.nets.len());
    println!("components:  {}", board.components.len());
    println!("padstacks:   {}", board.padstacks.len());
    println!("outline pts: {}", board.outline.len());
    if !warnings.is_empty() {
        println!("warnings:    {}", warnings.len());
    }
    Ok(())
}

fn run_route(
    input: &PathBuf,
    output: &PathBuf,
    max_time: u64,
    threads: usize,
    seed: u64,
) -> anyhow::Result<()> {
    let src = fs::read_to_string(input)?;
    let (mut board, warnings) = fr_dsn::read_board(&src);
    eprintln!(
        "loaded {}: {} layers, {} nets, {} components ({} warnings)",
        board.name,
        board.layer_count(),
        board.nets.len(),
        board.components.len(),
        warnings.len()
    );

    let opts = fr_engine::RouteOptions {
        max_time_secs: max_time, threads, seed,
        width: 0, clearance: 0, max_layers: 0,
    };
    let report = fr_engine::route_board(&mut board, &opts);
    eprintln!(
        "routed: {}/{} nets, {} traces, {} vias in {} passes",
        report.nets_completed, report.nets_total, board.traces.len(), board.vias.len(), report.passes
    );

    let ext = output.extension().and_then(|e| e.to_str()).unwrap_or("rte").to_lowercase();
    let out = if ext == "ses" {
        fr_dsn::write_ses(&board)
    } else {
        fr_dsn::write_rte(&board)
    };
    fs::write(output, out)?;
    eprintln!("wrote {}", output.display());
    Ok(())
}

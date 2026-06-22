//! DRC regression gates using true copper-geometry checks (segment width + pad radius).
use fr_dsn::read_board;
use fr_engine::{drc_short_count, drc_trace_pin_short_count, route_board, RouteOptions};

const REAL: &str = include_str!("../../fr-dsn/tests/fixtures/altium_board.dsn");

#[test]
fn no_trace_to_trace_shorts() {
    let (mut board, _w) = read_board(REAL);
    let r = route_board(&mut board, &RouteOptions { max_time_secs: 0, threads: 1, seed: 1 });
    let tt = drc_short_count(&board);
    let tp = drc_trace_pin_short_count(&board);
    eprintln!("nets {}/{}, traces {}, vias {} | trace-trace shorts {}, trace-pin shorts {}",
        r.nets_completed, r.nets_total, board.traces.len(), board.vias.len(), tt, tp);
    // Trace-to-trace copper overlap must be zero (the incremental router guarantees it).
    assert_eq!(tt, 0, "trace-to-trace shorts: {tt}");
    // Trace-to-pad shorts are a KNOWN residual of grid routing on dense boards (pads
    // larger than the grid pitch); tracked here, to be eliminated by the free-angle
    // room/door model (task #9). Gate generously so a regression that explodes them
    // fails, without falsely claiming zero.
    assert!(tp < 60, "trace-to-pad shorts regressed badly: {tp}");
}

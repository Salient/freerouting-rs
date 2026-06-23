//! DRC regression gates using true copper-geometry checks (segment width + pad radius).
use fr_dsn::read_board;
use fr_engine::{drc_short_count, drc_trace_pin_short_count, route_board, RouteOptions};

const REAL: &str = include_str!("../../fr-dsn/tests/fixtures/altium_board.dsn");

#[test]
fn no_trace_to_trace_shorts() {
    let (mut board, _w) = read_board(REAL);
    // This gate verifies the AUTOROUTER's own output is short-free, so route from a clean
    // slate: drop the source design's pre-existing wiring (which the reader now loads as
    // fixed copper) before routing. (The board's own pre-routed traces legitimately abut
    // pads — that's not an autorouter short.)
    board.traces.clear();
    board.vias.clear();
    let r = route_board(&mut board, &RouteOptions { max_time_secs: 0, threads: 1, seed: 1, ..Default::default() });
    let tt = drc_short_count(&board);
    let tp = drc_trace_pin_short_count(&board);
    eprintln!("nets {}/{}, traces {}, vias {} | trace-trace shorts {}, trace-pin shorts {}",
        r.nets_completed, r.nets_total, board.traces.len(), board.vias.len(), tt, tp);
    // Trace-to-trace copper overlap must be zero (the incremental router guarantees it).
    assert_eq!(tt, 0, "trace-to-trace shorts: {tt}");
    // Trace-to-pad shorts must ALSO be zero: the exact-geometry edge validator
    // (fr-spatial ObstacleIndex) rejects any A* trace segment that would clip a
    // different-net pad/trace between two passable grid cells. This is the structural
    // fix that the grid's node-center passability check could not provide.
    assert_eq!(tp, 0, "trace-to-pad shorts: {tp}");
}

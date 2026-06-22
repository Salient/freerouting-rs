//! DRC regression gate: route the real board and assert different-net trace overlaps
//! (shorts) stay at/near zero. The incremental router with width+clearance stamping
//! brought this from "many" overlaps down to a tiny residual (a single trace-corner
//! cell at the routing-grid resolution). We gate at a small threshold so regressions
//! that reintroduce widespread shorting fail loudly, while tracking the residual.
use fr_dsn::read_board;
use fr_engine::{drc_short_count, route_board, RouteOptions};

const REAL: &str = include_str!("../../fr-dsn/tests/fixtures/altium_board.dsn");

#[test]
fn routed_board_is_essentially_short_free() {
    let (mut board, _w) = read_board(REAL);
    let r = route_board(&mut board, &RouteOptions { max_time_secs: 0, threads: 1, seed: 1 });
    let shorts = drc_short_count(&board);
    eprintln!("nets {}/{}, traces {}, vias {}, shorts {}",
        r.nets_completed, r.nets_total, board.traces.len(), board.vias.len(), shorts);
    // Tolerance: a handful of single-cell corner touches at grid resolution are a known
    // residual to be eliminated by the free-angle room/door model. Widespread shorting
    // (the bug this guards) produced hundreds+.
    assert!(shorts <= 5, "too many different-net trace overlaps: {shorts}");
}

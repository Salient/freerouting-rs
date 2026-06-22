//! DRC regression gate: route the real board and assert there are NO different-net
//! trace overlaps (shorts). The incremental router (width+clearance stamping) plus
//! local-first net ordering produces a short-free result on this board; gate strictly
//! at 0 so any reappearance of shorting fails loudly.
use fr_dsn::read_board;
use fr_engine::{drc_short_count, route_board, RouteOptions};

const REAL: &str = include_str!("../../fr-dsn/tests/fixtures/altium_board.dsn");

#[test]
fn routed_board_is_short_free() {
    let (mut board, _w) = read_board(REAL);
    let r = route_board(&mut board, &RouteOptions { max_time_secs: 0, threads: 1, seed: 1 });
    let shorts = drc_short_count(&board);
    eprintln!("nets {}/{}, traces {}, vias {}, shorts {}",
        r.nets_completed, r.nets_total, board.traces.len(), board.vias.len(), shorts);
    assert_eq!(shorts, 0, "different-net trace overlaps (shorts): {shorts}");
}

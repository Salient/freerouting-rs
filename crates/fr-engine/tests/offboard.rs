//! Assert routed geometry stays inside the board outline.
use fr_dsn::read_board;
use fr_engine::{route_board, RouteOptions};
use fr_geometry::polygon_contains;
const REAL: &str = include_str!("../../fr-dsn/tests/fixtures/altium_board.dsn");
#[test]
fn no_routes_outside_outline() {
    let (mut board,_w)=read_board(REAL);
    // verify the AUTOROUTER's output stays in-bounds; route from a clean slate (drop the
    // source design's pre-existing wiring, now loaded as fixed copper).
    board.traces.clear();
    board.vias.clear();
    route_board(&mut board, &RouteOptions{max_time_secs:0,threads:1,seed:1,..Default::default()});
    let outline=board.outline.clone();
    let mut outside=0; let mut total=0;
    for t in &board.traces {
        for c in &t.corners { total+=1; if !polygon_contains(&outline,*c){outside+=1;} }
    }
    let mut via_out=0;
    for v in &board.vias { if !polygon_contains(&outline,v.location){via_out+=1;} }
    eprintln!("trace corners outside: {}/{}, vias outside: {}", outside, total, via_out);
    assert_eq!(outside,0,"{outside} trace corners outside the board outline");
    assert_eq!(via_out,0,"{via_out} vias outside the board outline");
}

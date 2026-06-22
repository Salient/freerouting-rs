use fr_dsn::read_board;
use fr_engine::{route_board, RouteOptions};
const REAL: &str = include_str!("../../fr-dsn/tests/fixtures/altium_board.dsn");
#[test]
fn vias_connect_traces() {
    let (mut board,_w)=read_board(REAL);
    route_board(&mut board, &RouteOptions{max_time_secs:0,threads:1,seed:1});
    let mut floating=0; let mut good=0;
    for v in &board.vias {
        // count traces of the SAME net with an endpoint at the via location
        let touching = board.traces.iter().filter(|t| t.net==v.net &&
            t.corners.first()==Some(&v.location) || t.corners.last()==Some(&v.location)).count();
        // a proper via has >=2 trace endpoints (one per layer) at its location
        let endpoints = board.traces.iter().filter(|t| t.net==v.net &&
            (t.corners.first()==Some(&v.location)||t.corners.last()==Some(&v.location))).count();
        let _=touching;
        if endpoints>=2 { good+=1 } else { floating+=1 }
    }
    eprintln!("vias: {} total, {} connect >=2 trace-ends, {} floating/underconnected", board.vias.len(), good, floating);
}

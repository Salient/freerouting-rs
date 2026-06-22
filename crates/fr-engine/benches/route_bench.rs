//! Benchmark: route the real Altium board with the incremental (short-free) router.
//! Run with `cargo bench -p fr-engine`.

use criterion::{criterion_group, criterion_main, Criterion};
use fr_dsn::read_board;
use fr_engine::{route_board, RouteOptions};

const REAL: &str = include_str!("../../fr-dsn/tests/fixtures/altium_board.dsn");

fn bench_route(c: &mut Criterion) {
    let mut group = c.benchmark_group("route_real_board");
    group.sample_size(10);
    group.bench_function("incremental", |b| {
        b.iter(|| {
            let (mut board, _) = read_board(REAL);
            let r = route_board(&mut board, &RouteOptions { max_time_secs: 0, threads: 1, seed: 1, ..Default::default() });
            criterion::black_box(r.nets_completed)
        })
    });
    group.finish();
}

criterion_group!(benches, bench_route);
criterion_main!(benches);

# freerouting-rs — Benchmark Report

Board: `crates/fr-dsn/tests/fixtures/altium_board.dsn` (real Altium export, 6 layers,
417 nets, 802 components, ~43k DSN lines). Routed to completion (no time cap).

Measured with `cargo bench -p fr-engine` (criterion, 10 samples) on this WSL2 host.

| Mode | Wall-clock (median) | Nets completed | Vias |
|------|---------------------|----------------|------|
| Sequential (`--threads 1`) | ~6.9 s | 326 / 417 | ~121 |
| Parallel (`--threads 0`, auto) | ~2.47 s | 372 / 417 | ~30 |

## Findings

- **~2.8x faster** wall-clock parallel vs sequential on this machine (CPU-bound; observed
  ~2 cores busy — the host exposes few cores to WSL).
- Parallel **also completes more nets** (372 vs 326) and uses **fewer vias**: Phase A
  routes every net against one clean obstacle snapshot, so each net finds an
  unobstructed (often via-free) path; the ordered-commit + sequential repair pass then
  resolves the overlaps. Sequential routing instead accumulates congestion as it goes.
- **Deterministic**: two parallel runs with the same seed/threads produce byte-identical
  output (verified by `cmp` and a unit test) - results are gathered into net-id order
  before committing and the repair pass is sequential, so output is independent of thread
  scheduling.

## Memory

Peak RSS was not rigorously profiled here; the index-arena/flat-array design keeps the
router's working set to a few large `Vec`s (per-search g-score + came-from arrays sized to
the grid, reused per connection) rather than the JVM's per-node object graph. A formal RSS
comparison against the Java tool is future work (Phase 9 acceptance item).

## How to reproduce

```
cargo bench -p fr-engine
# or wall-clock A/B:
./target/release/freerouting-rs route harness/sample_board.dsn -o /tmp/par.rte --threads 0
./target/release/freerouting-rs route harness/sample_board.dsn -o /tmp/seq.rte --threads 1
```

# freerouting-rs

A from-scratch, multithreaded **Rust** reimplementation of the freerouting PCB
autorouter, focused on a fast, deterministic engine and Altium-compatible Specctra
I/O. Reads a Specctra **DSN**, autoroutes it, and writes **RTE/SES** files that import
into Altium Designer.

## Build & test

```bash
cargo build --release          # CLI -> target/release/freerouting-rs
cargo test                     # all crates (fr-gui builds separately)
cargo bench -p fr-engine       # parallel vs sequential routing benchmark
(cd crates/fr-gui && cargo build --release)   # GUI (needs system X11/GL libs)
```

Requires a stable Rust toolchain. The GUI additionally needs X11/GL dev libraries
(`libx11-dev libxkbcommon-dev libgl1-mesa-dev libfontconfig1-dev …`).

## Usage

```bash
# summarize a board
freerouting-rs info BOARD.dsn

# route and write an Altium-importable route file (extension picks RTE/SES)
freerouting-rs route BOARD.dsn -o OUT.rte [--max-time SECS] [--threads N] [--seed S]

# headless render of the routed board to a PPM image (no window needed)
freerouting-rs-gui --render BOARD.dsn OUT.ppm [W H]

# interactive GUI (real display): open, pan/zoom, route, export
freerouting-rs-gui [BOARD.dsn]
```

`--threads 0` (default) uses the parallel scheduler; `--threads 1` forces sequential.
Output is deterministic for a given input + seed + thread count.

## Workspace

| Crate | Role |
|-------|------|
| `fr-geometry` | exact-integer points/vectors/orientation/box/convex tiles |
| `fr-spatial`  | R-tree spatial index (rstar) — for the future room/door model |
| `fr-board`    | board model: layers, padstacks (incl. shapeless), nets, items, rules |
| `fr-dsn`      | tolerant Specctra DSN reader + SES/RTE writers |
| `fr-route`    | grid weighted-A* router (obstacles, search, path→geometry) |
| `fr-engine`   | orchestration: pin placement, MST ordering, parallel scheduler |
| `fr-cli`      | `freerouting-rs` headless binary |
| `fr-gui`      | egui board viewer + autorouter front-end (built standalone) |

## Status & docs

- `STATUS.md` — current phase state and resume notes.
- `ACCEPTANCE.md` — acceptance-criteria results.
- `bench_report.md` — performance numbers.
- `../freerouting-rs-spec/` — the authoritative specification, the algorithm
  explanation, and the hard-won Altium-compatibility rules (`ALTIUM_COMPAT.md`).
- `harness/` — the Altium round-trip test harness (DelphiScript + driver).

## Altium compatibility (the non-obvious, verified rules)

Route files must be a **top-level `(routes …)` scope**, **CRLF** line endings, with each
wire/via on **one line** carrying its own `(net …)` and `(type …)`, and **scaled-integer**
coordinates. These were established by bisection against a live Altium importer and are
enforced by `crates/fr-dsn/tests/altium_validator.rs`. See `ALTIUM_COMPAT.md`.

## License

GPL-3.0-or-later (matching the original freerouting).

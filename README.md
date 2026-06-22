# freerouting-rs

A from-scratch, multithreaded **Rust** reimplementation of the freerouting PCB
autorouter, focused on a fast, deterministic engine and Altium-compatible Specctra
I/O. Reads a Specctra **DSN**, autoroutes it, and writes **RTE/SES** files that import
into Altium Designer.

## Build & test

```bash
cargo build --release          # core CLI -> target/release/freerouting-rs
cargo test                     # all crates (fr-gui builds separately)
cargo bench -p fr-engine       # parallel vs sequential routing benchmark
(cd crates/fr-gui && cargo build --release)   # main app (needs system X11/GL libs)
```

Requires a stable Rust toolchain. The GUI additionally needs X11/GL dev libraries
(`libx11-dev libxkbcommon-dev libgl1-mesa-dev libfontconfig1-dev …`).

## Usage

The main binary is `freerouting-rs-gui`: it launches the **interactive GUI by default**,
or batch-routes with `--headless` (no window/display needed).

```bash
# interactive GUI (default); optionally open a board on launch
freerouting-rs-gui [BOARD.dsn]
# under WSLg, force software GL so the window appears:
LIBGL_ALWAYS_SOFTWARE=1 freerouting-rs-gui BOARD.dsn

# headless batch route -> Altium-importable RTE/SES (extension picks the format;
# -o defaults to BOARD.rte next to the input)
freerouting-rs-gui --headless BOARD.dsn [-o OUT.rte] [--max-time SECS] [--threads N] [--seed S]

# headless render of the routed board to a PPM image (CI / no display)
freerouting-rs-gui --render OUT.ppm BOARD.dsn [--width W --height H]
```

`--threads 0` (default) uses the parallel scheduler; `--threads 1` forces sequential.
Output is deterministic for a given input + seed + thread count.

The core workspace also builds a smaller headless-only binary, `freerouting-rs`
(`fr-cli`), with `info` / `route` subcommands — useful when GUI system libs aren't
available (it excludes the GUI dependencies).

## Workspace

| Crate | Role |
|-------|------|
| `fr-geometry` | exact-integer points/vectors/orientation/box/convex tiles |
| `fr-spatial`  | R-tree spatial index (rstar) — for the future room/door model |
| `fr-board`    | board model: layers, padstacks (incl. shapeless), nets, items, rules |
| `fr-dsn`      | tolerant Specctra DSN reader + SES/RTE writers |
| `fr-route`    | grid weighted-A* router (obstacles, search, path→geometry) |
| `fr-engine`   | orchestration: pin placement, MST ordering, parallel scheduler |
| `fr-cli`      | `freerouting-rs` headless-only binary (`info`/`route`; no GUI deps) |
| `fr-gui`      | `freerouting-rs-gui` — main app: GUI by default, `--headless`/`--render` for batch (built standalone) |

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

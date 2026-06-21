# freerouting-rs task runner
build:
    cargo build
test:
    cargo test
bench:
    cargo bench
# build the GUI crate (needs system X11/GL libs; Phase 7)
gui:
    cargo build --manifest-path crates/fr-gui/Cargo.toml
# round-trip an Altium DSN through the router and back (Phase 4+)
roundtrip dsn out:
    cargo run -q --bin freerouting-rs -- route {{dsn}} -o {{out}}

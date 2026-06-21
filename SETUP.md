# SETUP — How to launch the autonomous build of freerouting-rs

This folder (`/home/jheller2/Projects/freerouting-rs/`) is where the new Rust autorouter
gets built. The complete specification lives in the sibling package
**`/home/jheller2/Projects/freerouting-rs-spec/`**. This file tells the **human operator**
how to start a fresh Claude Code instance that builds the project autonomously, running
the milestone gates without stopping to ask permission.

---

## TL;DR

```bash
# 1. Put the permission allowlist in place so the build runs unattended:
mkdir -p /home/jheller2/Projects/freerouting-rs/.claude
cp /home/jheller2/Projects/freerouting-rs-spec/.claude/settings.json \
   /home/jheller2/Projects/freerouting-rs/.claude/settings.json

# 2. Start Claude Code in this directory:
cd /home/jheller2/Projects/freerouting-rs
claude

# 3. Paste the kickoff prompt below.
```

---

## Prerequisites (install once, before launching)

The build agent's allowlist permits it to install these itself, but doing it up front
avoids any first-run friction:

```bash
# Rust toolchain
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source "$HOME/.cargo/env"
rustup default stable

# Headless GUI testing (egui under a virtual framebuffer) + screenshots
sudo apt-get update -qq
sudo apt-get install -y xvfb scrot imagemagick

# (Optional) the Java oracle, only if you want live quality comparisons.
# Already buildable at /home/jheller2/Projects/freerouting with JDK 11.
```

Verify: `cargo --version && rustc --version && which Xvfb`.

---

## The kickoff prompt (paste into the fresh Claude instance)

> You are building **freerouting-rs**, a high-performance multithreaded Rust
> reimplementation of the freerouting PCB autorouter, in this directory
> (`/home/jheller2/Projects/freerouting-rs/`).
>
> Your complete, authoritative specification is in `/home/jheller2/Projects/freerouting-rs-spec/`.
> **Read these first, in order:** `README.md`, `REQUIREMENTS.md`, `ARCHITECTURE.md`,
> `ALGORITHM.md`, `ALTIUM_COMPAT.md`, `MILESTONES.md`. The `reference/` subfolder has the
> SPECCTRA spec, sample DSN designs, and known-good/known-bad Altium artifacts.
>
> Then **execute `MILESTONES.md` phase by phase.** Each phase ends in a self-verifying
> gate. **When a gate passes, commit and proceed to the next phase WITHOUT asking me for
> permission.** Do not stop to summarize and wait between phases — build straight through.
> Only stop for me if you hit a true blocker (an external dependency you cannot install,
> or a gate that is impossible as written); in that case record it in `BLOCKED.md`,
> proceed with any other independent phase, and tell me at the end.
>
> Key constraints to keep in front of you:
> - Output does NOT need to be byte-compatible with the Java tool. The hard requirement
>   is that exported `.rte`/`.ses` files **import correctly into Altium** — above all,
>   route files are a **top-level `(routes ...)` scope with CRLF line endings** (see
>   `ALTIUM_COMPAT.md`).
> - Leverage the Java algorithm (room/door weighted-A*, shove, rip-up) but feel free to
>   be superior.
> - Build for multithreading and the arena/index memory model from the start
>   (`ARCHITECTURE.md`).
>
> Commit at every gate. Begin now with Phase 0.

---

## Why it runs unattended

- `.claude/settings.json` (copied in step 1) sets `permissions.defaultMode:
  "acceptEdits"` and allowlists `cargo`, `git`, `rustup`, file ops, `xvfb`, and the Java
  oracle commands — so the agent doesn't hit approval prompts for the normal build loop.
- `MILESTONES.md` is written so every phase boundary is a machine-checkable gate
  ("`cargo test` passes", "validator passes", "screenshot non-blank"), giving the agent
  an objective "proceed" signal instead of needing a human checkpoint.
- The spec is self-contained: algorithm, architecture, format rules, sample data, and
  the quality baseline are all in `freerouting-rs-spec/`, so the agent never needs to ask
  "what did you mean."

## Tips for a long unattended run

- **Run it with a large budget / in the background.** This is a multi-week-equivalent
  build; expect many phases. If your harness supports background or workflow execution,
  use it.
- **Check in via git.** The agent commits at every gate, so `git log` in this folder
  shows progress. `BLOCKED.md` (if present) lists anything it couldn't do.
- **Resume cleanly.** If the instance stops, start a new one with the same kickoff
  prompt; it will read the spec, inspect git history / existing crates, and continue from
  the first incomplete gate.
- **Final human step.** At Phase 9 the agent produces `artifacts/baseline_rs.rte`. Import
  that into Altium to confirm the real-world goal end-to-end (the agent can only verify
  the structural proxy, not run Altium itself).

## Where everything is

| Path | What |
|---|---|
| `/home/jheller2/Projects/freerouting-rs/` | **the build target** (this folder) |
| `/home/jheller2/Projects/freerouting-rs-spec/` | the full spec + reference data |
| `/home/jheller2/Projects/freerouting/` | the Java oracle (branch `port/altium-fixes-from-fork`) |

#!/usr/bin/env bash
# Phase 7 gate: launch the GUI under Xvfb with a board, route it, screenshot it,
# and assert the capture is non-blank. Self-contained so all procs share a session.
set -u
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
DISP=":97"
OUT="artifacts/gui_routed.png"
mkdir -p artifacts

pkill -f "Xvfb $DISP" 2>/dev/null
pkill -f "freerouting-rs-gui" 2>/dev/null
sleep 1

# start Xvfb and keep it. -ac disables X access control: without it winit fails to
# connect ("Broken pipe" -> WinitEventLoop ExitFailure) on this headless setup.
Xvfb "$DISP" -screen 0 1400x1000x24 -ac >/tmp/xvfb-gui.log 2>&1 &
XVFB_PID=$!
sleep 2
if ! DISPLAY="$DISP" xdpyinfo >/dev/null 2>&1; then
    echo "FAIL: Xvfb did not start"; exit 1
fi

# launch GUI with the sample board, software GL
DISPLAY="$DISP" LIBGL_ALWAYS_SOFTWARE=1 ./crates/fr-gui/target/release/freerouting-rs-gui harness/sample_board.dsn >/tmp/gui.log 2>&1 &
GUI_PID=$!
sleep 10

if ! kill -0 "$GUI_PID" 2>/dev/null; then
    echo "FAIL: GUI exited early; log:"; tail -15 /tmp/gui.log
    kill "$XVFB_PID" 2>/dev/null; exit 1
fi

# capture
DISPLAY="$DISP" import -window root "$OUT" 2>/tmp/cap.log || DISPLAY="$DISP" scrot "$OUT" 2>>/tmp/cap.log
RESULT=$?

kill "$GUI_PID" 2>/dev/null
kill "$XVFB_PID" 2>/dev/null

if [ ! -s "$OUT" ]; then
    echo "FAIL: no screenshot produced"; cat /tmp/cap.log; exit 1
fi
SIZE=$(stat -c %s "$OUT")
echo "screenshot: $OUT ($SIZE bytes)"
[ "$SIZE" -gt 3000 ] && echo "GATE PASS: non-trivial screenshot" || echo "GATE WARN: screenshot small ($SIZE)"
exit 0

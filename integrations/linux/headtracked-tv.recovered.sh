#!/usr/bin/env bash
GAME_DIR="$HOME/Games/Recompiled/Mario-Kart-Wii-VR"
TRACKER_DIR="$HOME/mkw-headtracker"
PYTHON="$TRACKER_DIR/runtime/cpython-3.12.14-linux-x86_64-gnu/bin/python3.12"
LOG="$TRACKER_DIR/tracker.log"
export PYTHONPATH="$TRACKER_DIR/runtime/site-packages${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONUNBUFFERED=1

exec 9>"$TRACKER_DIR/session.lock"
flock -n 9 || { echo "Mario Kart head tracking is already running"; exit 1; }
if [[ -z "$MKW_CAMERA_DEVICE" ]]; then
    for candidate in /dev/v4l/by-id/*C270*video-index0 /dev/v4l/by-id/*video-index0 /dev/video0; do
        if [[ -e "$candidate" ]]; then export MKW_CAMERA_DEVICE="$candidate"; break; fi
    done
fi
if [[ ! -e "$MKW_CAMERA_DEVICE" ]]; then
    echo "Connect the tracking webcam before launching Mario Kart."
    command -v notify-send >/dev/null && notify-send "Mario Kart head tracking" "Connect the tracking webcam and launch again."
    exit 1
fi
# Keep the C270 delivering the requested 30 FPS; its low-light dynamic
# exposure previously halved capture rate. Match local UK mains lighting.
if command -v v4l2-ctl >/dev/null; then
    v4l2-ctl -d "$MKW_CAMERA_DEVICE" \
        --set-ctrl=exposure_dynamic_framerate=0,power_line_frequency=1 \
        2>>"$LOG" || true
fi
TRACKER_PID=""
GAME_PID=""
cleanup() {
    if [[ -n "$TRACKER_PID" ]]; then kill "$TRACKER_PID" 2>/dev/null; wait "$TRACKER_PID" 2>/dev/null; fi
    if [[ -n "$GAME_PID" ]]; then kill "$GAME_PID" 2>/dev/null; fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM
start_tracker() {
    "$PYTHON" "$TRACKER_DIR/headtracker-v3.py" >>"$LOG" 2>&1 &
    TRACKER_PID=$!
}
cd "$TRACKER_DIR" || exit 1
start_tracker
sleep 3
if ! kill -0 "$TRACKER_PID" 2>/dev/null; then tail -40 "$LOG"; exit 1; fi
cd "$GAME_DIR" || exit 1
./WiiCompiled >>"$GAME_DIR/game.log" 2>&1 &
GAME_PID=$!
while kill -0 "$GAME_PID" 2>/dev/null; do
    if ! kill -0 "$TRACKER_PID" 2>/dev/null; then
        wait "$TRACKER_PID" 2>/dev/null
        start_tracker
    fi
    sleep 2
done
wait "$GAME_PID"
RESULT=$?
GAME_PID=""
exit "$RESULT"

#!/usr/bin/env python3
"""Home Assistant's existing SSH control interface; updates one atomic TOML file."""
import fcntl
import math
import sys
import time
import tomllib
from pathlib import Path

ROOT = Path.home() / "mkw-headtracker"
CONFIG = ROOT / "tuning.toml"
DEFAULTS = dict(yaw_gain=2.0, pitch_gain=1.0, roll_gain=1.0,
                x_gain=1.0, y_gain=1.0, z_gain=1.0,
                smoothing=1.0, prediction_ms=0.0,
                recenter_request=0, save_calibration_request=0,
                reset_calibration_request=0)

def main():
    ROOT.mkdir(parents=True, exist_ok=True)
    with (ROOT / "control.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        try:
            values = DEFAULTS | tomllib.loads(CONFIG.read_text()).get("headtracking", {})
        except FileNotFoundError:
            values = DEFAULTS.copy()
        if sys.argv[1] == "apply" and len(sys.argv) == 10:
            numbers = list(map(float, sys.argv[2:]))
            if not all(math.isfinite(n) for n in numbers):
                raise ValueError("Controls must be finite numbers")
            if not 0 <= numbers[6] <= 1 or not 0 <= numbers[7] <= 100:
                raise ValueError("Smoothing is 0–1; prediction is 0–100 ms")
            values.update(zip(list(DEFAULTS)[:8], numbers))
        elif sys.argv[1] == "request" and len(sys.argv) == 3:
            key = {"recenter": "recenter_request", "save": "save_calibration_request",
                   "reset": "reset_calibration_request"}[sys.argv[2]]
            values[key] = max(time.time_ns() // 1_000_000, int(values[key]) + 1)
        elif sys.argv[1] != "init":
            raise ValueError("Use apply with eight values, request recenter/save/reset, or init")
        temporary = CONFIG.with_suffix(".tmp")
        temporary.write_text("[headtracking]\n" + "".join(f"{k} = {v}\n" for k, v in values.items()))
        temporary.replace(CONFIG)
        print("Head-tracking controls saved")

if __name__ == "__main__":
    main()

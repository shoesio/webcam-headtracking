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
            if not 0 <= numbers[6] <= 5 or not 0 <= numbers[7] <= 100:
                raise ValueError("Smoothing is 0–5; prediction is 0–100 ms")
            values.update(zip(list(DEFAULTS)[:8], numbers))
        elif sys.argv[1] == "request" and len(sys.argv) == 3:
            key = {"recenter": "recenter_request", "save": "save_calibration_request",
                   "reset": "reset_calibration_request"}[sys.argv[2]]
            values[key] = max(time.time_ns() // 1_000_000, int(values[key]) + 1)
        elif sys.argv[1] == "training" and len(sys.argv) == 3:
            label = {'stop':0,'overarm':1,'underarm':2,'normal':3}[sys.argv[2]]
            values.update(training_label=label,
                          training_request=time.time_ns() // 1_000_000)
        elif sys.argv[1] == "steering" and len(sys.argv) == 6:
            enabled, strength, smoothing, invert = map(float, sys.argv[2:])
            if (not all(math.isfinite(v) for v in (enabled,strength,smoothing,invert))
                    or enabled not in (0,1) or invert not in (0,1)
                    or not .5 <= strength <= 50 or not 0 <= smoothing <= 500):
                raise ValueError('Steering: enabled/invert 0/1, strength 0.5–50, smoothing 0–500 ms')
            values.update(steering_enabled=int(enabled), steering_gain=strength,
                          steering_smoothing_ms=smoothing, steering_invert=int(invert))
        elif sys.argv[1] == "steering-curve" and len(sys.argv) == 3:
            values['steering_curve'] = {'Linear':0,'Gentle':1,'Strong':2}[sys.argv[2]]
        elif sys.argv[1] == "steering-center" and len(sys.argv) == 2:
            values['steering_center_request'] = time.time_ns() // 1_000_000
        elif sys.argv[1] == "throws" and len(sys.argv) == 5:
            enabled, stroke, visibility = map(float, sys.argv[2:])
            if (not all(math.isfinite(v) for v in (enabled,stroke,visibility))
                    or enabled not in (0,1) or not .1 <= stroke <= 1
                    or not .1 <= visibility <= .9):
                raise ValueError('Throws: enabled 0/1, stroke .1–1, visibility .1–.9')
            values.update(throw_enabled=int(enabled),throw_stroke=stroke,throw_visibility=visibility)
        elif sys.argv[1] == "lean" and len(sys.argv) == 6:
            enabled, activation, release, confirmation = map(float, sys.argv[2:])
            if (not all(math.isfinite(v) for v in (enabled, activation, release, confirmation))
                    or enabled not in (0, 1) or not 3 <= activation <= 40
                    or not 0 <= release < activation or not 0 <= confirmation <= 500):
                raise ValueError("Lean: enabled 0/1, activation 3–40°, release below activation, confirmation 0–500 ms")
            values.update(lean_enabled=int(enabled), lean_activation_deg=activation,
                          lean_release_deg=release, lean_confirmation_ms=confirmation)
        elif sys.argv[1] != "init":
            raise ValueError("Use apply with eight values, request recenter/save/reset, or init")
        temporary = CONFIG.with_suffix(".tmp")
        temporary.write_text("[headtracking]\n" + "".join(f"{k} = {v}\n" for k, v in values.items()))
        temporary.replace(CONFIG)
        print("Head-tracking controls saved")

if __name__ == "__main__":
    main()

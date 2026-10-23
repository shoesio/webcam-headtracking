# Webcam head tracking

The webcam head/gesture tracking companion used by **Mario Kart VR: Headtracked**
on the BC-250 and Steam Deck. Source checkpoint recovered from the BC-250 on
23 October 2026, preserving its six earlier Git checkpoints and the current
uncommitted improvements. The live machines were not modified for this upload.

This is the **tracker side**, not the game. It does not contain WiiCompiled,
Mario Kart game assets, ROMs, NAND/Mii/save data, texture packs or the modified
game's camera/OpenXR implementation. The recovered game executable needs to be
preserved separately: vanilla WiiCompiled does not acquire head tracking just
by running this sender. See [scope and recovery](docs/RECOVERY.md).

## Included

- Webcam face pose from MediaPipe, with independent yaw/pitch/roll and XYZ gains.
- Continuous mapping without spatial dead zones or fixed head-pose caps.
- Measurement smoothing, optional rotation-jump rejection, a short temporal
  reconstruction stage and configurable prediction.
- Live TOML tuning, recenter, calibration save/reset, health statistics and a
  private annotated preview for Home Assistant.
- Lean-to-drift, forward/backward item throws, numeric gesture-training tools,
  and the experimental hand-steering prototype with its tests.

## Layout

| File | Responsibility |
| --- | --- |
| `headtracker-v3.py` | Capture, face inference, pose mapping, live config, preview and 120 Hz UDP sender |
| `adaptive_pose.py` | Fixed median/low-pass filtering; filename is historical |
| `rotation_guard.py` | Optional one-sample confirmation of large rotational jumps |
| `pose_diagnostics.py` | Opt-in numeric pose traces |
| `lean_gesture.py` | Lean recognition and additive X11 keyboard drift input |
| `arm_throw.py`, `throw_sequence.py` | Body inference, directional throws and key pulses |
| `throw_training.py`, `analyse_throw_training.py` | Optional numeric recordings and offline analysis |
| `wheel_gesture.py`, `wheel_virtual.py`, `wheel_profile.py` | Experimental hand steering; not required for head tracking |
| `ha_control.py`, `ha_status.py` | Atomic tuning writes and status/preview readout |
| `integrations/` | Recovered Linux launcher and sanitized Home Assistant examples |

## Setup on Linux

The recovered code expects its checkout at `~/mkw-headtracker`, with Python 3.12
recommended for the original deployment. It uses Linux V4L2, `/dev/shm`, and
X11/XWayland plus `libX11`/`libXtst` for gesture key injection. It is not a
cross-platform desktop application. Camera and input permissions must be granted
by the host; running as root is not required or recommended.

```sh
git clone https://github.com/shoesio/webcam-headtracking.git ~/mkw-headtracker
cd ~/mkw-headtracker
python3.12 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

Dependencies are intentionally not a reproducible lockfile. The recovered host
reported MediaPipe 1.0.1, NumPy 2.5.3 and OpenCV packages 5.0.0.93; fresh package
installation has not been validated against that bundled runtime. Keep the
existing runtime when restoring the working game installation.

Obtain the models separately from Google's official model guides:

- [Face Landmarker](https://developers.google.com/edge/mediapipe/solutions/vision/face_landmarker): save the model beside `headtracker-v3.py` as `face_landmarker.task`.
- [Pose Landmarker Lite](https://developers.google.com/edge/mediapipe/solutions/vision/pose_landmarker): optionally save as `pose_landmarker_lite.task` to enable the arm worker.

Models and bundled Python are not redistributed here. Consult their upstream
terms before redistributing them. Do not add third-party game artwork/binaries.

`tuning.toml` contains the BC-250 settings at backup time. For a new installation,
start with the conservative head-only example instead:

```sh
cp examples/tuning.head-only.toml tuning.toml
MKW_CAMERA_DEVICE=/dev/video0 .venv/bin/python headtracker-v3.py
```

Use the actual camera's `/dev/v4l/by-id/...-video-index0` path where possible.
The recovered game's Linux launcher is preserved under `integrations/linux/` as
an integration reference, **not a universal installer**. It assumes the original
game path and bundled runtime and applies UK 50 Hz anti-flicker settings.

## Live controls

From the checkout, with the tracker running:

```sh
.venv/bin/python ha_control.py apply 2 2 1 0 0 0 0.5 0
.venv/bin/python ha_control.py request recenter
.venv/bin/python ha_control.py request save
.venv/bin/python ha_control.py request reset
.venv/bin/python ha_status.py
```

`apply` arguments are yaw, pitch, roll, X, Y, Z gain, smoothing and prediction
milliseconds. Commands operate on `~/mkw-headtracker/tuning.toml`, not arbitrary
current directories. Changes are hot-reloaded; a game rebuild is not needed.
Keep the original `ha-control`/`ha-status` wrappers for the bundled deployment,
or use the Python commands above for a `.venv` installation.

The "start calibration" UI control is a recenter/reference operation, not an
automatic look-at-screen-target calibration wizard. Gains determine scale;
recenter does not establish a head movement boundary.

Smoothing 0 bypasses measurement smoothing and sender reconstruction; nonzero
values use the fixed filter and a roughly 20 ms reconstruction stage. Values
above 1 deliberately allow very slow smoothing. More prediction can overshoot;
it is not a replacement for good camera measurements. See [design lessons](docs/DESIGN.md).

## Tests

The synthetic tests do not open a webcam or inject game input:

```sh
python3 -m pip install numpy
python3 -m unittest discover -p 'test_*.py' -v
```

Recorded-gesture replay is skipped when private training files are absent.
Passing synthetic tests is not a claim that a fresh hardware deployment has
been tested. Source tests were run locally at export; the live tracker/game
was not restarted or modified.

## Privacy and licensing

Preview JPEGs, landmark/pose traces, training recordings, runtime files, SSH
keys, Home Assistant secrets and game files are excluded. Keep the preview
behind authenticated HA access, not in its public `www` directory. The optional
virtual wheel can affect Steam controller ordering; it is disabled by default
in the head-only example. Do not enable it merely to use webcam head tracking.

This is a private preservation repository. No new open-source license is being
assigned by this export; upstream components retain their own licenses.

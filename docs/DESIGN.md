# Design and lessons learned

## Spatial mapping versus temporal reconstruction

The head camera uses proportional, independently adjustable six-axis gain.
There are no spatial dead zones, fixed pose caps or response-compression curve.
Calibration is a reference pose and scale, not a maximum allowed head movement.
Head tracking must be applied on top of the game's own moving camera, rather
than stabilizing away banking, steering, bumps or first-person behavior.

Measurement noise and low update frequency are separate problems. The webcam
capture and face inference run independently of the 120 Hz sender. Each tracker
result is processed once. A short causal output reconstruction stage reduces
visible sample-to-sample stepping, but cannot recover unmeasured motion.

`FixedPoseFilter` uses a three-result causal median followed by a fixed
first-order low-pass. Zero smoothing bypasses both stages; at nonzero settings
the median itself adds roughly a sample of delay. Smoothing <=1 uses
`tau = 0.020 + 0.480 * amount**2` seconds; above 1 uses `tau = 0.500 * amount`.
The sender adds roughly 20 ms of reconstruction only when smoothing is enabled.
The filename `adaptive_pose.py` is historical: this version is deliberately not
speed-adaptive. Heavier smoothing sacrifices response time.

Prediction uses recent filtered velocity and a 0–100 ms configurable horizon,
decayed as a sample ages. The recovered baseline uses 0 ms: prediction was not
universally beneficial and could amplify wobble. Keep it configurable.

The optional `RotationGuard` checks abrupt measurement changes, not absolute
pose limits. An unexpected large rotation is held for one result, then sustained
movement is accepted. Fast turns may still need tuning; face detection loss is
not fixed by raising prediction. Existing guard enable marker is preserved.

## Camera and diagnostics

A strong face-detection quality score is not proof of low pose jitter. Looking
sideways, a small face far from the camera, lighting/exposure and occlusion can
make pose noisy despite a good "lock". Use the annotated preview and numeric
diagnostics together. C270 dynamic low-light frame rate previously halved
capture to 15 FPS; exposure settings and UK 50 Hz flicker compensation helped
restore 30 FPS. Those settings are camera/region specific.

Freezing all head pose axes removed the observed jitter during diagnosis, which
separated tracking noise from the game's own kart-body animation. Do not alter
the original game camera to disguise a noisy input signal.

## Input and gestures

Keyboard injection is additive and does not create a controller. Separate
features must own separate keys: releasing a throw-direction key that is also
held for throttle can cancel acceleration indefinitely because each producer
maintains its own held-state cache.

Throwing needs wind-up, stroke and release/rearm phases, not a hand-raise trigger.
The included recognizer and tests are tuned for the earlier user's motions;
private recordings are intentionally not uploaded. Lean button activation has
gesture hysteresis, distinct from head-camera mapping, which has no dead zone.

Experimental hand steering compares wrist image heights with manual gain and a
selectable ramp. It holds the last steering value on tracking loss, by user
request. That is a game prototype, not a safe control policy for machinery.
The optional virtual controller can interfere with Steam navigation/order and
was rolled back for normal play; do not enable it by default.

Steam may hide the real wheel and expose a virtual pad. Inspect actual device
routing before repeatedly changing button bindings; raw evdev numbering and
SDL game-controller numbering are different. This repository does not change
Steam's controller configuration automatically.

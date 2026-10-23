import cv2
import mediapipe as mp
import numpy as np
import socket
import time
import math
import threading
import sys
import subprocess
import json
import os
import signal
from pathlib import Path
from adaptive_pose import FixedPoseFilter
from pose_diagnostics import diagnostics
from rotation_guard import RotationGuard
from lean_gesture import LeanGesture, DriftKeyboard
from arm_throw import FrameBus

DEVICE = os.environ.get("MKW_CAMERA_DEVICE", "/dev/video1")
MODEL = str(Path(__file__).with_name("face_landmarker.task"))

def stop_on_signal(signum, frame):
    raise KeyboardInterrupt

signal.signal(signal.SIGTERM, stop_on_signal)

UDP_HOST = "127.0.0.1"
UDP_PORT = 26760

CAPTURE_W = 640
CAPTURE_H = 480
CAPTURE_FPS = 30

CROP_X1 = 0.25
CROP_X2 = 0.75
CROP_Y1 = 0.15
CROP_Y2 = 0.85

INFERENCE_WIDTH = 320
CALIBRATION_SAMPLES = 20

# Configurable pure-pose defaults. Calibration is only the neutral reference,
# never a limit or a dead zone.
TUNING_FILE = Path.home() / "mkw-headtracker" / "tuning.toml"
STATUS_FILE = Path.home() / "mkw-headtracker" / "status.json"
PREVIEW_FILE = Path.home() / "mkw-headtracker" / "preview.jpg"
last_preview_time = 0.0
CALIBRATION_FILE = Path.home() / ".local/share/WiiCompiled/HeadTrackingCalibration.toml"
YAW_GAIN = 2.0
PITCH_GAIN = 1.0
ROLL_GAIN = 1.0
X_GAIN = 1.0
Y_GAIN = 1.0
Z_GAIN = 1.0
DEADZONE = 0.0
CURVE = 1.0
SMOOTH = 0.65
PREDICTION_MS = 30.0
OUTLIER_REJECTION = False
GUARD_SWITCH = Path.home() / "mkw-headtracker" / "outlier_rejection.enabled"
LEAN_ENABLED = False
LEAN_ACTIVATION = 12.0
LEAN_RELEASE = 6.0
LEAN_CONFIRMATION_MS = 120.0
latest_lean_roll = 0.0
latest_lean_time = 0.0
lean_gesture = LeanGesture()
drift_keyboard = DriftKeyboard()
lean_error = ""

FRAME_BYTES = CAPTURE_W * CAPTURE_H * 3
arm_bus = FrameBus(writer=True)
arm_process = None

BaseOptions = mp.tasks.BaseOptions
Vision = mp.tasks.vision
FaceLandmarker = Vision.FaceLandmarker
FaceLandmarkerOptions = Vision.FaceLandmarkerOptions
RunningMode = Vision.RunningMode

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

pose_lock = threading.Lock()
frame_lock = threading.Lock()
target_lock = threading.Lock()
output_stop = threading.Event()

latest_target_pose = np.zeros(6, dtype=np.float64)
latest_target_velocity = np.zeros(6, dtype=np.float64)
latest_target_time = 0.0

latest_matrix = None
latest_result_time = 0.0
tracking_count = 0
callback_count = 0

latest_frame = None
latest_frame_seq = 0
capture_count = 0
capture_dead = False


def write_status(payload):
    """Publish a small atomic health snapshot for Home Assistant."""
    try:
        STATUS_FILE.parent.mkdir(parents=True, exist_ok=True)
        tmp = STATUS_FILE.with_suffix(".tmp")
        tmp.write_text(json.dumps(payload, separators=(",", ":")) + "\n")
        tmp.replace(STATUS_FILE)
    except OSError:
        pass


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def continuous_axis(value, gain):
    return value * gain


def _read_toml_section(path, section):
    values = {}
    current = None
    try:
        for line in path.read_text().splitlines():
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            if line.startswith("[") and line.endswith("]"):
                current = line[1:-1].strip()
                continue
            if current != section or "=" not in line:
                continue
            key, value = (part.strip() for part in line.split("=", 1))
            values[key] = float(value)
    except (OSError, ValueError):
        return {}
    return values


def _read_saved_calibration():
    return _read_toml_section(CALIBRATION_FILE, "calibration")


def _save_calibration(values):
    try:
        CALIBRATION_FILE.parent.mkdir(parents=True, exist_ok=True)
        CALIBRATION_FILE.write_text(
            "[calibration]\n" +
            "\n".join(f"{key} = {value:.9f}" for key, value in values.items()) +
            "\n"
        )
    except OSError as exc:
        print(f"Unable to save head calibration: {exc}")


def load_live_tuning(last_mtime, last_check, now):
    """Reload mapping values and calibration commands without restarting."""
    global YAW_GAIN, PITCH_GAIN, ROLL_GAIN, X_GAIN, Y_GAIN, Z_GAIN
    global DEADZONE, CURVE, SMOOTH, PREDICTION_MS
    global pending_recenter, pending_save, pending_reset
    global last_recenter_request, last_save_request, last_reset_request
    global OUTLIER_REJECTION
    global LEAN_ENABLED, LEAN_ACTIVATION, LEAN_RELEASE, LEAN_CONFIRMATION_MS

    if now - last_check < 0.20:
        return last_mtime, last_check
    last_check = now
    enabled = GUARD_SWITCH.exists()
    if enabled != OUTLIER_REJECTION:
        rotation_guard.reset()
        OUTLIER_REJECTION = enabled
        print(f"Experimental rotation outlier rejection: {enabled}")
    try:
        mtime = TUNING_FILE.stat().st_mtime_ns
    except OSError:
        return last_mtime, last_check
    if mtime == last_mtime:
        return last_mtime, last_check

    values = _read_toml_section(TUNING_FILE, "headtracking")
    YAW_GAIN = values.get("yaw_gain", YAW_GAIN)
    PITCH_GAIN = values.get("pitch_gain", PITCH_GAIN)
    ROLL_GAIN = values.get("roll_gain", ROLL_GAIN)
    X_GAIN = values.get("x_gain", X_GAIN)
    Y_GAIN = values.get("y_gain", Y_GAIN)
    Z_GAIN = values.get("z_gain", Z_GAIN)
    DEADZONE = 0.0
    CURVE = 1.0
    SMOOTH = clamp(values.get("smoothing", SMOOTH), 0.0, 5.0)
    PREDICTION_MS = clamp(values.get("prediction_ms", PREDICTION_MS), 0.0, 100.0)
    LEAN_ENABLED = bool(values.get("lean_enabled", 0))
    LEAN_ACTIVATION = clamp(values.get("lean_activation_deg", 12.0), 3.0, 40.0)
    LEAN_RELEASE = clamp(values.get("lean_release_deg", 6.0), 0.0, LEAN_ACTIVATION - 0.5)
    LEAN_CONFIRMATION_MS = clamp(values.get("lean_confirmation_ms", 120.0), 0.0, 500.0)

    recenter = int(values.get("recenter_request", last_recenter_request))
    save = int(values.get("save_calibration_request", last_save_request))
    reset = int(values.get("reset_calibration_request", last_reset_request))
    if last_recenter_request != -1 and recenter != last_recenter_request:
        pending_recenter = True
        last_recenter_request = recenter
    if last_save_request != -1 and save != last_save_request:
        pending_save = True
        last_save_request = save
    if last_reset_request != -1 and reset != last_reset_request:
        pending_reset = True
        last_reset_request = reset
    last_recenter_request = recenter
    last_save_request = save
    last_reset_request = reset

    print(
        "LIVE HEAD TRACKING: "
        f"gains=({YAW_GAIN:.2f},{PITCH_GAIN:.2f},{ROLL_GAIN:.2f}; "
        f"{X_GAIN:.2f},{Y_GAIN:.2f},{Z_GAIN:.2f}) "
        f"smoothing={SMOOTH:.2f} prediction={PREDICTION_MS:.0f}ms"
    )
    return mtime, last_check



def pose_sender():
    """Send a continuously filtered, short-horizon predicted six-axis pose."""
    global smooth_yaw, smooth_pitch, smooth_roll
    global smooth_x, smooth_y, smooth_z
    global lean_error

    previous = time.monotonic()
    smooth = np.zeros(6, dtype=np.float64)
    while not output_stop.is_set():
        now = time.monotonic()
        dt = clamp(now - previous, 0.001, 0.10)
        previous = now

        with target_lock:
            target = latest_target_pose.copy()
            velocity = latest_target_velocity.copy()
            age = 0.0 if latest_target_time == 0.0 else now - latest_target_time
            lean_roll, lean_time = latest_lean_roll, latest_lean_time

        try:
            if LEAN_ENABLED:
                drift_keyboard.open()
            held = lean_gesture.update(lean_roll, lean_time, now, LEAN_ENABLED,
                                       LEAN_ACTIVATION, LEAN_RELEASE, LEAN_CONFIRMATION_MS)
            drift_keyboard.set(held)
            lean_error = ""
        except OSError as exc:
            lean_error = str(exc)
            lean_gesture.held = False
            try:
                drift_keyboard.close()
            except OSError:
                pass

        prediction = (PREDICTION_MS / 1000.0) * math.exp(-max(age, 0.0) / 0.060)
        predicted = target + velocity * prediction

        response = clamp(SMOOTH, 0.0, 5.0)
        if response <= 0.001:
            # Zero smoothing means genuinely direct pose output.
            follow = 1.0
        else:
            # The live amount controls FixedPoseFilter at measurement rate.
            # Keep reconstruction short so filtering does not stack extra lag.
            # Measurement filtering handles noise; this short stage reconstructs
            # a continuous output between the slower camera samples.
            smoothing_tau = 0.020
            follow = 1.0 - math.exp(-dt / smoothing_tau)
        smooth += (predicted - smooth) * follow

        smooth_yaw, smooth_pitch, smooth_roll = smooth[:3]
        smooth_x, smooth_y, smooth_z = smooth[3:]

        try:
            sock.sendto(
                (" ".join(f"{value:.4f}" for value in smooth)).encode(),
                (UDP_HOST, UDP_PORT)
            )
            diagnostics.record("sent", pose=smooth.tolist(), sample_age_ms=age * 1000.0)
        except OSError:
            break
        time.sleep(1.0 / 120.0)


def matrix_to_angles(M):
    R = np.asarray(M, dtype=np.float64)[:3, :3]

    sy = math.sqrt(
        R[0, 0] * R[0, 0] +
        R[1, 0] * R[1, 0]
    )

    if sy > 1e-6:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0.0

    return (
        math.degrees(y), math.degrees(x), math.degrees(z),
        float(M[0, 3]), float(M[1, 3]), float(M[2, 3]),
    )


def on_result(result, output_image, timestamp_ms):
    global last_preview_time
    global latest_matrix
    global latest_result_time
    global tracking_count
    global callback_count

    now = time.monotonic()
    # Publish the result immediately: drawing/encoding the HA preview must
    # not delay the game pose. The main loop consumes it independently of
    # whether another camera frame is available for submission.
    with pose_lock:
        callback_count += 1
        if result.facial_transformation_matrixes:
            latest_matrix = np.array(result.facial_transformation_matrixes[0], copy=True)
            latest_result_time = now
            tracking_count += 1

    if now - last_preview_time >= 0.5:
        last_preview_time = now
        # Reuse capture frames; never open a second camera or block inference
        # on network transfers. HA pulls the latest atomic JPEG independently.
        with frame_lock:
            preview = None if latest_frame is None else latest_frame.copy()
        if preview is not None:
            h, w = preview.shape[:2]
            try:
                arms = json.loads(Path(__file__).with_name('arm_status.json').read_text())
                points = arms.get('arm_landmarks', [])
                if time.time()-arms.get('updated_at',0) < 2 and len(points) >= 17:
                    for a,b in [(11,12),(11,13),(13,15),(12,14),(14,16)]:
                        if min(points[a][3],points[b][3]) >= .25:
                            pa=(int(points[a][0]*w),int(points[a][1]*h))
                            pb=(int(points[b][0]*w),int(points[b][1]*h))
                            cv2.line(preview,pa,pb,(240,100,255),2)
                            cv2.circle(preview,pb,4,(240,100,255),-1)
                cv2.putText(preview,
                    f"Throws: F {arms.get('forward_throws',0)} / B {arms.get('backward_throws',0)} | {arms.get('last_throw','')}",
                    (8,h-32),cv2.FONT_HERSHEY_SIMPLEX,.45,(240,100,255),1)
            except (OSError,ValueError,TypeError):
                pass
            x1, x2 = int(w * CROP_X1), int(w * CROP_X2)
            y1, y2 = int(h * CROP_Y1), int(h * CROP_Y2)
            cv2.rectangle(preview, (x1,y1), (x2,y2), (255,180,0), 1)
            label = "No face detected"
            if result.face_landmarks and result.facial_transformation_matrixes:
                landmarks = result.face_landmarks[0]
                points = [(int(x1+p.x*(x2-x1)), int(y1+p.y*(y2-y1))) for p in landmarks]
                xs, ys = zip(*points)
                lo, hi = (min(xs), min(ys)), (max(xs), max(ys))
                cv2.rectangle(preview, lo, hi, (80,240,80), 2)
                for p in points[::8]:
                    cv2.circle(preview, p, 1, (80,240,80), -1)
                matrix = np.asarray(result.facial_transformation_matrixes[0])
                angles = matrix_to_angles(matrix)[:3]
                origin = points[1]
                for axis, color in enumerate([(0,0,255),(0,255,0),(255,120,0)]):
                    end = (int(origin[0]+35*matrix[0,axis]), int(origin[1]-35*matrix[1,axis]))
                    cv2.arrowedLine(preview, origin, end, color, 2, tipLength=0.2)
                label = f"Face {hi[0]-lo[0]}x{hi[1]-lo[1]}px | yaw {angles[0]:+.1f} pitch {angles[1]:+.1f} roll {angles[2]:+.1f}"
            cv2.rectangle(preview,(0,0),(w,52),(20,20,20),-1)
            cv2.putText(preview,label,(8,20),cv2.FONT_HERSHEY_SIMPLEX,0.43,(255,255,255),1)
            cv2.putText(preview,"Blue box: tracking crop | axes: red X, green Y, blue Z",(8,42),cv2.FONT_HERSHEY_SIMPLEX,0.4,(220,220,220),1)
            cv2.putText(preview,time.strftime("%H:%M:%S UTC",time.gmtime()),(8,h-10),cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,255,255),1)
            ok, jpeg = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                try:
                    temporary = PREVIEW_FILE.with_suffix(".tmp.jpg")
                    temporary.write_bytes(jpeg.tobytes())
                    temporary.replace(PREVIEW_FILE)
                except OSError:
                    pass

options = FaceLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODEL),
    running_mode=RunningMode.LIVE_STREAM,
    num_faces=1,
    min_face_detection_confidence=0.20,
    min_face_presence_confidence=0.25,
    min_tracking_confidence=0.35,
    output_facial_transformation_matrixes=True,
    result_callback=on_result,
)


# GStreamer owns V4L2. We ask it to output fixed-size raw BGR frames
# to stdout. No OpenCV VideoCapture anywhere.
gst_cmd = [
    "gst-launch-1.0",
    "-q",

    "v4l2src",
    f"device={DEVICE}",

    "!",

    "video/x-raw,format=YUY2,"
    f"width={CAPTURE_W},height={CAPTURE_H},"
    f"framerate={CAPTURE_FPS}/1",

    "!",

    # Keep latency bounded. If downstream falls behind, discard.
    "queue",
    "leaky=downstream",
    "max-size-buffers=1",

    "!",

    "videoconvert",

    "!",

    "video/x-raw,format=BGR",

    "!",

    "fdsink",
    "fd=1",
    "sync=false",
]


print("Opening webcam through GStreamer...")
print(
    f"Request: {CAPTURE_W}x{CAPTURE_H} "
    f"YUY2 @ {CAPTURE_FPS}fps"
)

try:
    gst = subprocess.Popen(
        gst_cmd,
        stdout=subprocess.PIPE,
        stderr=None,
        bufsize=0,
    )
except Exception as e:
    print(f"ERROR starting GStreamer: {e}")
    sys.exit(1)


def read_exact(stream, size):
    chunks = []
    remaining = size

    while remaining:
        data = stream.read(remaining)

        if not data:
            return None

        chunks.append(data)
        remaining -= len(data)

    return b"".join(chunks)


def capture_worker():
    global latest_frame
    global latest_frame_seq
    global capture_count
    global capture_dead

    while True:
        data = read_exact(gst.stdout, FRAME_BYTES)

        if data is None:
            break

        frame = np.frombuffer(
            data,
            dtype=np.uint8
        ).reshape(
            CAPTURE_H,
            CAPTURE_W,
            3
        ).copy()
        arm_bus.publish(data)

        with frame_lock:
            latest_frame = frame
            latest_frame_seq += 1
            capture_count += 1

    capture_dead = True


capture_thread = threading.Thread(
    target=capture_worker,
    name="gst-camera",
    daemon=True,
)

capture_thread.start()

print("Waiting for first frame...")

deadline = time.monotonic() + 5.0

while True:
    with frame_lock:
        ready = latest_frame is not None

    if ready:
        break

    if gst.poll() is not None:
        print(
            "ERROR: GStreamer exited before producing a frame "
            f"(status {gst.returncode})."
        )
        sys.exit(1)

    if time.monotonic() > deadline:
        print("ERROR: timed out waiting for camera.")
        gst.terminate()
        sys.exit(1)

    time.sleep(0.01)


print("✓ Camera streaming")
print(f"MediaPipe inference width: {INFERENCE_WIDTH}")
print()
print("Look straight at the TV for calibration.")
print()

centre_samples = []
centre_yaw = None
centre_pitch = None
centre_roll = None
centre_x = None
centre_y = None
centre_z = None
saved = _read_saved_calibration()
if saved:
    centre_yaw = saved.get("yaw")
    centre_pitch = saved.get("pitch")
    centre_roll = saved.get("roll")
    centre_x = saved.get("x")
    centre_y = saved.get("y")
    centre_z = saved.get("z")

pending_recenter = False
pending_save = False
pending_reset = False
last_recenter_request = -1
last_save_request = -1
last_reset_request = -1

smooth_yaw = 0.0
smooth_pitch = 0.0

submitted_frames = 0

last_capture_count = 0
last_submitted_frames = 0
last_tracking_count = 0
last_callback_count = 0

last_report = time.monotonic()
jitter_estimate = 0.0
last_timestamp_ms = 0
last_seq = 0
last_pose_sample = -1
pose_filter = FixedPoseFilter()
rotation_guard = RotationGuard()
live_tuning_mtime = -1
live_tuning_check = 0.0
output_thread = threading.Thread(target=pose_sender, daemon=True)
output_thread.start()
if Path(__file__).with_name('pose_landmarker_lite.task').exists():
    arm_process = subprocess.Popen([sys.executable, str(Path(__file__).with_name('arm_throw.py'))])


try:
    with FaceLandmarker.create_from_options(options) as landmarker:

        while True:
            now = time.monotonic()
            live_tuning_mtime, live_tuning_check = load_live_tuning(
                live_tuning_mtime, live_tuning_check, now
            )

            if capture_dead:
                print("\nERROR: camera stream stopped.")
                break

            with frame_lock:
                if (
                    latest_frame is None
                    or latest_frame_seq == last_seq
                ):
                    frame = None
                    seq = last_seq
                else:
                    frame = latest_frame.copy()
                    seq = latest_frame_seq

            if frame is None:
                time.sleep(0.001)
            else:
                last_seq = seq
                h, w = frame.shape[:2]
                x1, x2 = int(w * CROP_X1), int(w * CROP_X2)
                y1, y2 = int(h * CROP_Y1), int(h * CROP_Y2)
                crop = frame[y1:y2, x1:x2]
                ch, cw = crop.shape[:2]
                infer_h = max(1, int(ch * INFERENCE_WIDTH / float(cw)))
                small = cv2.resize(crop, (INFERENCE_WIDTH, infer_h),
                                   interpolation=cv2.INTER_AREA)
                rgb = cv2.cvtColor(small, cv2.COLOR_BGR2RGB)
                mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
                timestamp_ms = max(int(time.monotonic() * 1000), last_timestamp_ms + 1)
                last_timestamp_ms = timestamp_ms
                landmarker.detect_async(mp_image, timestamp_ms)
                submitted_frames += 1

            with pose_lock:
                pose_sample = tracking_count
                pose_sample_time = latest_result_time
                M = (
                    None
                    if latest_matrix is None
                    else np.array(
                        latest_matrix,
                        copy=True
                    )
                )

                pose_age = (
                    999.0
                    if latest_result_time == 0
                    else
                    time.monotonic()
                    - latest_result_time
                )

            raw_yaw = None
            raw_pitch = None
            raw_roll = None
            raw_x = None
            raw_y = None
            raw_z = None

            if M is not None and pose_age < 0.35 and pose_sample != last_pose_sample:
                last_pose_sample = pose_sample

                raw_yaw, raw_pitch, raw_roll, raw_x, raw_y, raw_z = (
                    matrix_to_angles(M)
                )

                if pending_reset:
                    pose_filter.reset()
                    rotation_guard.reset()
                    latest_lean_time = 0.0
                    centre_yaw = centre_pitch = centre_roll = None
                    centre_x = centre_y = centre_z = None
                    centre_samples.clear()
                    pending_reset = False
                    try:
                        CALIBRATION_FILE.unlink()
                    except FileNotFoundError:
                        pass
                    print("Head calibration reset; hold still for recentering.")

                if pending_recenter:
                    pose_filter.reset()
                    rotation_guard.reset()
                    latest_lean_time = 0.0
                    centre_yaw, centre_pitch, centre_roll = raw_yaw, raw_pitch, raw_roll
                    centre_x, centre_y, centre_z = raw_x, raw_y, raw_z
                    centre_samples.clear()
                    pending_recenter = False
                    with target_lock:
                        latest_target_pose[:] = 0.0
                        latest_target_velocity[:] = 0.0
                    print("Head tracking recentered.")

                if pending_save and centre_yaw is not None:
                    _save_calibration({"yaw": centre_yaw, "pitch": centre_pitch, "roll": centre_roll,
                                       "x": centre_x, "y": centre_y, "z": centre_z})
                    pending_save = False
                    print(f"Head calibration saved to {CALIBRATION_FILE}")

                if (centre_yaw is None or centre_pitch is None or centre_roll is None or
                        centre_x is None or centre_y is None or centre_z is None):

                    centre_samples.append(
                        (raw_yaw, raw_pitch, raw_roll, raw_x, raw_y, raw_z)
                    )

                    if (
                        len(centre_samples)
                        >= CALIBRATION_SAMPLES
                    ):
                        centre_yaw = sum(
                            p[0]
                            for p in centre_samples
                        ) / len(centre_samples)

                        centre_pitch = sum(p[1] for p in centre_samples) / len(centre_samples)
                        centre_roll = sum(p[2] for p in centre_samples) / len(centre_samples)
                        centre_x = sum(p[3] for p in centre_samples) / len(centre_samples)
                        centre_y = sum(p[4] for p in centre_samples) / len(centre_samples)
                        centre_z = sum(p[5] for p in centre_samples) / len(centre_samples)

                        print(
                            "\nCENTRE LOCKED: "
                            f"yaw={centre_yaw:+.2f} pitch={centre_pitch:+.2f} "
                            f"roll={centre_roll:+.2f}"
                        )
                        print(
                            "🔥 HEAD TRACKING ACTIVE"
                        )
                        print()

                else:

                    measured_angles = [raw_yaw, raw_pitch, raw_roll]
                    yaw, pitch, roll = (
                        rotation_guard.update(measured_angles, pose_sample_time)
                        if OUTLIER_REJECTION else measured_angles
                    )
                    target = np.array([
                        continuous_axis(-(yaw - centre_yaw), YAW_GAIN),
                        continuous_axis(pitch - centre_pitch, PITCH_GAIN),
                        continuous_axis(roll - centre_roll, ROLL_GAIN),
                        continuous_axis(raw_x - centre_x, X_GAIN),
                        continuous_axis(raw_y - centre_y, Y_GAIN),
                        continuous_axis(raw_z - centre_z, Z_GAIN),
                    ], dtype=np.float64)
                    sample_time = pose_sample_time
                    mapped_pose = target.copy()
                    target = pose_filter.update(target, sample_time, SMOOTH)
                    diagnostics.record(
                        "measurement", sample_time=sample_time,
                        raw=[raw_yaw, raw_pitch, raw_roll, raw_x, raw_y, raw_z],
                        mapped=mapped_pose.tolist(), filtered=target.tolist(),
                        smoothing=SMOOTH, prediction_ms=PREDICTION_MS,
                        gains=[YAW_GAIN, PITCH_GAIN, ROLL_GAIN, X_GAIN, Y_GAIN, Z_GAIN],
                        outlier_rejection=OUTLIER_REJECTION,
                        rejected_rotation_axes=rotation_guard.rejected,
                    )
                    with target_lock:
                        latest_lean_roll = float(roll - centre_roll)
                        latest_lean_time = sample_time
                        if latest_target_time > 0.0:
                            sample_dt = max(sample_time - latest_target_time, 1.0 / 120.0)
                            measured_velocity = (target - latest_target_pose) / sample_dt
                            velocity_residual = measured_velocity - latest_target_velocity
                            jitter_speed = float(np.linalg.norm(velocity_residual[:3]))
                            jitter_estimate += (
                                jitter_speed - jitter_estimate
                            ) * clamp(sample_dt / 0.20, 0.05, 1.0)
                            # Prediction must not turn single-frame webcam jitter into
                            # visible wobble. Filter velocity separately from pose so
                            # smoothing remains responsive without amplifying noise.
                            velocity_follow = clamp(sample_dt / 0.080, 0.08, 1.0)
                            latest_target_velocity[:] += (
                                measured_velocity - latest_target_velocity
                            ) * velocity_follow
                        else:
                            latest_target_velocity[:] = 0.0
                        latest_target_pose[:] = target
                        latest_target_time = sample_time

            now = time.monotonic()

            if now - last_report >= 1.0:

                elapsed = now - last_report

                with frame_lock:
                    captured_now = capture_count

                with pose_lock:
                    tracked_now = tracking_count
                    callbacks_now = callback_count

                cap_fps = (
                    captured_now
                    - last_capture_count
                ) / elapsed

                sub_fps = (
                    submitted_frames
                    - last_submitted_frames
                ) / elapsed

                mp_fps = (
                    callbacks_now
                    - last_callback_count
                ) / elapsed

                face_fps = (
                    tracked_now
                    - last_tracking_count
                ) / elapsed

                if M is None or pose_age >= 0.35:
                    lock_status = "lost"
                    lock_quality = 0
                elif (centre_yaw is None or centre_pitch is None or centre_roll is None or
                      centre_x is None or centre_y is None or centre_z is None):
                    lock_status = "calibrating"
                    lock_quality = 35
                else:
                    # Face-rate and pose age are deliberately conservative proxies
                    # for lock quality; the tracker does not expose a stable ML score.
                    rate_score = clamp(face_fps / max(cap_fps, 1.0), 0.0, 1.0)
                    age_score = clamp(1.0 - (pose_age / 0.35), 0.0, 1.0)
                    lock_quality = round(100.0 * (0.65 * rate_score + 0.35 * age_score))
                    lock_status = (
                        "good" if lock_quality >= 80 else
                        "weak" if lock_quality >= 45 else
                        "lost"
                    )

                write_status({
                    "status": lock_status,
                    "quality": int(lock_quality),
                    "pose_age_ms": round(max(pose_age, 0.0) * 1000.0, 1),
                    "capture_fps": round(cap_fps, 1),
                    "inference_fps": round(mp_fps, 1),
                    "face_fps": round(face_fps, 1),
                    "jitter_deg_s": round(jitter_estimate, 1),
                    "output_velocity_deg_s": round(float(np.linalg.norm(latest_target_velocity[:3])), 1),
                    "updated_at": time.time(),
                    "lean_enabled": LEAN_ENABLED,
                    "lean_roll_deg": round(latest_lean_roll, 1),
                    "lean_drift_held": lean_gesture.held,
                    "lean_error": lean_error,
                })

                if M is None or pose_age >= 0.35:
                    pose_text = "NO FACE"

                elif (centre_yaw is None or centre_pitch is None or centre_roll is None or
                      centre_x is None or centre_y is None or centre_z is None):
                    pose_text = (
                        f"CAL "
                        f"{len(centre_samples):02d}/"
                        f"{CALIBRATION_SAMPLES}"
                    )

                else:
                    # Reports can fall between fresh results now; use the
                    # latest matrix for display without filtering it twice.
                    raw_yaw, raw_pitch, raw_roll = matrix_to_angles(M)[:3]
                    pose_text = (
                        f"RAW yaw {raw_yaw:+6.1f} "
                        f"pitch {raw_pitch:+6.1f} "
                        f"roll {raw_roll:+6.1f} | "
                        f"OUT {smooth_yaw:+6.1f} "
                        f"{smooth_pitch:+6.1f}"
                    )

                print(
                    f"GST {cap_fps:4.1f} | "
                    f"SUB {sub_fps:4.1f} | "
                    f"MP {mp_fps:4.1f} | "
                    f"FACE {face_fps:4.1f} | "
                    f"{pose_text}"
                )

                last_capture_count = captured_now
                last_submitted_frames = (
                    submitted_frames
                )
                last_tracking_count = tracked_now
                last_callback_count = callbacks_now
                last_report = now


except KeyboardInterrupt:
    print("\nStopping...")


finally:
    output_stop.set()
    if arm_process is not None:
        arm_process.terminate()
        try:
            arm_process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            arm_process.kill()
            arm_process.wait()
    try:
        output_thread.join(timeout=1.0)
    except Exception:
        pass
    drift_keyboard.close()

    try:
        sock.sendto(
            b"0 0",
            (UDP_HOST, UDP_PORT)
        )
    except Exception:
        pass

    try:
        gst.terminate()
        gst.wait(timeout=2)
    except Exception:
        try:
            gst.kill()
        except Exception:
            pass

    sock.close()

    print("Mario recentered.")

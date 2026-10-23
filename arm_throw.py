"""Independent upper-body inference and quick-rearming throw gestures."""
import ctypes
import json
import math
import mmap
import os
from pathlib import Path
import signal
import struct
import sys
import time
import tomllib
from throw_training import TrainingRecorder
from throw_sequence import ThrowRecognizer
from wheel_gesture import WheelGesture, WheelBridge, WheelOutput

ROOT = Path(__file__).parent
FRAME_PATH = Path('/dev/shm') / f'mkw-arm-frame-{os.getuid()}'
FRAME_BYTES = 640 * 480 * 3


class FrameBus:
    def __init__(self, writer=False):
        self.writer = writer
        flags = os.O_RDWR | (os.O_CREAT | os.O_TRUNC if writer else 0) | os.O_NOFOLLOW
        self.fd = os.open(FRAME_PATH, flags, 0o600)
        if writer:
            os.ftruncate(self.fd, FRAME_BYTES + 16)
        self.mem = mmap.mmap(self.fd, FRAME_BYTES + 16)
        self.seq = 0

    def publish(self, data):
        self.seq += 2
        struct.pack_into('<Q', self.mem, 0, self.seq - 1)
        self.mem[16:] = data
        struct.pack_into('<d', self.mem, 8, time.monotonic())
        struct.pack_into('<Q', self.mem, 0, self.seq)

    def read(self):
        seq, timestamp = struct.unpack_from('<Qd', self.mem, 0)
        if not seq or seq % 2:
            return None
        data = self.mem[16:]
        if seq != struct.unpack_from('<Q', self.mem, 0)[0]:
            return None
        return seq, timestamp, data

    def close(self):
        self.mem.close()
        os.close(self.fd)


class LegacyThrowRecognizer:
    def __init__(self):
        self.arms = {}
        self.last_fire = -10.0

    def reset(self):
        self.arms.clear()

    def update(self, points, now, stroke=0.25, minimum_visibility=0.4):
        """Shoulder-normalised wrist strokes; one event then a new wind-up."""
        if not points:
            self.reset()
            return None
        width = math.hypot(points[11][0]-points[12][0], points[11][1]-points[12][1])
        if width < 0.03:
            self.reset()
            return None
        event = None
        for side, s, e, w in [('left',11,13,15), ('right',12,14,16)]:
            if min(points[i][3] for i in (s,e,w)) < minimum_visibility:
                self.arms.pop(side, None)
                continue
            y = (points[w][1] - points[s][1]) / width
            upper = [points[s][i]-points[e][i] for i in range(3)]
            lower = [points[w][i]-points[e][i] for i in range(3)]
            norm = math.sqrt(sum(v*v for v in upper)*sum(v*v for v in lower))
            angle = math.degrees(math.acos(max(-1,min(1,
                sum(a*b for a,b in zip(upper,lower))/max(norm,1e-9)))))
            reach = math.sqrt(sum((points[w][i]-points[s][i])**2 for i in range(3)))/width
            old = self.arms.get(side)
            mode = old['mode'] if old else None
            dt = now-old['time'] if old else 0
            if not old or dt > 0.25:
                self.arms[side] = dict(mode=None, time=now, y=y, origin=y,
                                      low_origin=y, prep_angle=angle, prep_reach=reach)
                continue
            velocity = (y-old['y'])/max(dt, 0.001)
            origin = old['origin']
            low_origin = min(old['low_origin'],y)
            prep_angle, prep_reach = old['prep_angle'],old['prep_reach']
            state = dict(time=now,y=y,origin=origin,low_origin=low_origin,
                         prep_angle=prep_angle,prep_reach=prep_reach)
            # A spent stroke cannot rearm merely by continuing its extension.
            # Drawing back to the same preparation pose rearms immediately.
            if mode == 'spent_forward' and not (y < -0.25 and velocity < -0.5):
                self.arms[side] = dict(mode=mode,**state)
                continue
            if mode == 'spent_backward' and not (y > 0.85 and velocity > 0.5):
                self.arms[side] = dict(mode=mode,**state)
                continue
            extension = angle-prep_angle >= 12 or reach-prep_reach >= .15
            firing = ((mode == 'forward' and y-origin >= stroke and velocity > 1.0 and extension)
                      or (mode == 'backward' and origin-y >= stroke and velocity < -1.0))
            if firing and now-self.last_fire >= 0.12 and event is None:
                event = mode
                self.last_fire = now
                mode = 'spent_' + mode
                low_origin = y
            elif y < -0.25:
                if mode != 'forward':
                    origin = y
                    prep_angle, prep_reach = angle,reach
                mode = 'forward'
                origin = min(origin, y)
                if velocity < 0:
                    prep_angle = min(prep_angle,angle)
                    prep_reach = min(prep_reach,reach)
            elif y > 0.85 and (mode == 'backward' or
                               (velocity > .5 and y-low_origin >= stroke)):
                # Merely starting with a hand resting low is NOT a wind-up.
                # Observe a deliberate downward draw first, then its reversal.
                if mode != 'backward':
                    origin = y
                mode = 'backward'
                origin = max(origin, y)
            self.arms[side] = dict(mode=mode, time=now, y=y, origin=origin,
                                  low_origin=low_origin,prep_angle=prep_angle,prep_reach=prep_reach)
        return event


class ItemKeyboard:
    def __init__(self):
        from lean_gesture import DriftKeyboard
        self.connection = DriftKeyboard()
        self.connection.open()
        c = self.connection
        self.item = c.x.XKeysymToKeycode(c.display, 0xffc6)  # F9
        self.up = c.x.XKeysymToKeycode(c.display, 0xffc3)    # F6
        self.down = c.x.XKeysymToKeycode(c.display, 0xffc4)  # F7
        self.active = set()
        self.release_at = 0
        self.item_at = 0
        self.item_pressed = False

    def key(self, code, held):
        c = self.connection
        if not c.xtest.XTestFakeKeyEvent(c.display, code, int(held), 0):
            raise OSError('Throw key rejected by game display')
        c.x.XFlush(c.display)
        if held:
            self.active.add(code)
        else:
            self.active.discard(code)

    def throw(self, direction, now):
        if self.active:
            return False
        self.key(self.up if direction == 'forward' else self.down, True)
        self.item_at = now + 0.035
        self.release_at = now + 0.12
        self.item_pressed = False
        return True

    def tick(self, now):
        if self.active and now >= self.release_at:
            self.release()
        elif self.active and not self.item_pressed and now >= self.item_at:
            self.key(self.item, True)
            self.item_pressed = True

    def release(self):
        # Item up first, direction second.
        for code in [self.item, self.up, self.down]:
            if code in self.active:
                self.key(code, False)
        self.item_pressed = False

    def close(self):
        try:
            self.release()
        finally:
            self.connection.close()


def extend_bindings(path):
    """Add F9 item and F6/F7 stick Y; retain all existing keyboard bindings."""
    path = Path(path)
    data = bytearray(path.read_bytes())
    if len(data) != 740 or struct.unpack_from('<II',data) != (0x444e424b,3):
        raise ValueError('Unsupported keyboard bindings format')
    # Each port is: bool, 12 eight-byte button bindings, 10 axis bindings.
    base = 33
    for slot, code in [(6,66)]:  # GC L / item
        old = struct.unpack_from('<i',data,base+slot*8)[0]
        if old not in (-1,code):
            raise ValueError('Item keyboard binding already occupied')
        struct.pack_into('<i',data,base+slot*8,code)
    axes = base+96
    for slot,code in [(2,63),(3,64),(8,66)]:
        old = struct.unpack_from('<i',data,axes+slot*8)[0]
        if old not in (-1,code):
            raise ValueError('Throw axis keyboard binding already occupied')
        struct.pack_into('<i',data,axes+slot*8,code)
        struct.pack_into('<h',data,axes+slot*8+6,1)
    temporary = path.with_suffix('.throw.tmp')
    temporary.write_bytes(data)
    temporary.replace(path)


def main():
    import cv2
    import mediapipe as mp
    import numpy as np
    os.nice(5)
    bus = FrameBus()
    recognizer = ThrowRecognizer()
    wheel = WheelGesture(interpolation_ms=30)
    wheel_bridge = WheelBridge(ROOT)
    wheel_output = WheelOutput(wheel,wheel_bridge)
    training = TrainingRecorder(ROOT)
    keyboard = None
    last_seq = 0
    last_status = 0
    last_config = 0
    config = {}
    counts = dict(forward=0, backward=0)
    last_throw = ''
    points = []
    inference_count = 0
    started = time.monotonic()
    parent = os.getppid()
    def stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    options = mp.tasks.vision.PoseLandmarkerOptions(
        base_options=mp.tasks.BaseOptions(model_asset_path=str(ROOT/'pose_landmarker_lite.task')),
        running_mode=mp.tasks.vision.RunningMode.VIDEO, num_poses=1,
        min_pose_detection_confidence=0.35, min_pose_presence_confidence=0.35,
        min_tracking_confidence=0.35)
    next_frame = 0
    try:
        with mp.tasks.vision.PoseLandmarker.create_from_options(options) as model:
            while os.getppid() == parent:
                now = time.monotonic()
                if now-last_config > .2:
                    last_config = now
                    try:
                        config = tomllib.loads((ROOT/'tuning.toml').read_text()).get('headtracking',{})
                    except (OSError,ValueError):
                        config = {}
                training.update(config,now)
                wheel.set_enabled(bool(config.get('steering_enabled',0)) and not training.mode)
                enabled = bool(config.get('throw_enabled',0)) and not training.mode
                if keyboard:
                    if not enabled:
                        keyboard.release()
                    else:
                        keyboard.tick(now)
                sample = bus.read() if now >= next_frame else None
                if sample and sample[0] != last_seq and now-sample[1] < .2:
                    last_seq, stamp, data = sample
                    next_frame = now + .05  # independent body tracker, up to 20 FPS
                    frame = np.frombuffer(data,dtype=np.uint8).reshape(480,640,3)
                    result = model.detect_for_video(
                        mp.Image(image_format=mp.ImageFormat.SRGB,
                                 data=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)),int(stamp*1000))
                    inference_count += 1
                    points = ([[p.x,p.y,p.z,p.visibility] for p in result.pose_landmarks[0]]
                              if result.pose_landmarks else [])
                    world = ([[p.x,p.y,p.z,p.visibility] for p in result.pose_world_landmarks[0]]
                             if result.pose_world_landmarks else [])
                    training.record(points,world,stamp,time.monotonic())
                    wheel.update(points,stamp,config if not training.mode else {},
                                 received_time=time.monotonic())
                    event = recognizer.update(points,stamp,
                              stroke=float(config.get('throw_stroke',.25)),
                              minimum_visibility=float(config.get('throw_visibility',.4))) if enabled else None
                    if not enabled:
                        recognizer.reset()
                    if event:
                        wheel.freeze_for_throw(stamp)
                        if keyboard is None:
                            keyboard = ItemKeyboard()
                        if keyboard.throw(event,time.monotonic()):
                            counts[event] += 1
                            last_throw = event
                            print(f'THROW {event} {counts}',flush=True)
                elif sample and now-sample[1] >= .2:
                    recognizer.reset()
                    wheel.update([],now,config)
                    points = []
                    if keyboard:
                        keyboard.release()
                if now-last_status > .25:
                    last_status = now
                    payload = dict(throw_enabled=enabled, arm_visible=bool(points),
                                   arm_landmarks=points, last_throw=last_throw,
                                   forward_throws=counts['forward'],backward_throws=counts['backward'],
                                   arm_fps=round(inference_count/max(now-started,.1),1),
                                   updated_at=time.time())
                    payload.update(training.status(now))
                    payload.update(wheel.status())
                    payload.update(wheel_bridge.status())
                    temp = ROOT/'arm_status.tmp'
                    temp.write_text(json.dumps(payload))
                    temp.replace(ROOT/'arm_status.json')
                time.sleep(.003)
    except KeyboardInterrupt:
        pass
    finally:
        wheel_output.close()
        wheel_bridge.close()
        training.close()
        if keyboard:
            keyboard.close()
        bus.close()


if __name__ == '__main__':
    main()

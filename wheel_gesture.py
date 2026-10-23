"""Two-wrist wheel angle, throw/loss hold, and analogue pad bridge."""
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import threading
from functools import wraps


def synchronized(method):
    @wraps(method)
    def call(self,*args,**kwargs):
        with self.lock:
            return method(self,*args,**kwargs)
    return call


def steering_ramp(value, mode=0):
    """Instantaneous curve with a nonzero centre slope and full-lock endpoints."""
    value = max(-1., min(1., value))
    blend = {0:0.,1:.6,2:.85}.get(mode,0.)
    return (1-blend)*value + blend*value**3


class WheelGesture:
    def __init__(self, interpolation_ms=0):
        self.lock = threading.RLock()
        self.interpolation_ms = interpolation_ms
        self.transition_at = 0.
        self.transition_from = 0.
        self.interpolating = False
        self.angle = None
        self.center = 0.
        self.center_request = 0
        self.target = self.value = 0.
        self.valid_at = -10.
        self.previous = None
        self.enabled = False
        self.smoothing = .08
        self.reason = 'disabled'
        self.grip_spacing = None
        self.neutral_height = 0.
        self.height_delta = 0.
        self.freeze_until = 0.
        self.curve = 0

    @synchronized
    def set_enabled(self, enabled):
        self.enabled = enabled

    @synchronized
    def update(self, points, stamp, config, received_time=None):
        self.enabled = bool(config.get('steering_enabled', 0))
        self.smoothing = float(config.get('steering_smoothing_ms', 80))/1000
        self.interpolation_ms = max(0., min(100., float(config.get(
            'steering_interpolation_ms',self.interpolation_ms))))
        if not self.enabled:
            self.target = self.value = 0.
            self.interpolating = False
            self.reason = 'disabled'
            return
        if len(points) < 33:
            self.target = self.value
            self.interpolating = False
            self.reason = 'holding last steering — hands lost'
            return
        left, right = points[15], points[16]
        spacing = math.hypot((left[0]-right[0])*4/3, left[1]-right[1])
        # User's wheel can obscure the wrists: use inferred positions without
        # gating on visibility. Missing poses still hold the previous command.
        # Only reject effectively coincident points, not their wheel posture.
        holding = spacing > .005
        request = int(config.get('steering_center_request', 0))
        if stamp < self.freeze_until:
            self.target = self.value
            self.interpolating = False
            self.reason = 'holding last steering — item throw'
            return
        if not holding:
            self.target = self.value
            self.interpolating = False
            self.reason = 'holding last steering — gesture / uncertain hands'
            return
        # Keep the oriented line continuous through vertical and angle wrap.
        angle = math.degrees(math.atan2(left[1]-right[1], (left[0]-right[0])*4/3))
        if self.angle is not None:
            while angle-self.angle > 180:
                angle -= 360
            while angle-self.angle < -180:
                angle += 360
        self.angle = angle
        if request != self.center_request:
            self.center = angle
            self.center_request = request
            self.grip_spacing = spacing
        elif self.grip_spacing is None:
            self.grip_spacing = spacing
        direction = -1 if not config.get('steering_invert', 0) else 1
        # Manual strength only: no angle conversion or hand-spacing calibration.
        self.height_delta = left[1]-right[1]
        strength = max(.5,min(50.,float(config.get('steering_gain',20.))))
        self.curve = int(config.get('steering_curve',0))
        self.target = steering_ramp(direction*self.height_delta*strength,self.curve)
        self.transition_from = self.value
        self.transition_at = stamp if received_time is None else received_time
        self.interpolating = self.interpolation_ms > 0
        self.valid_at = stamp
        self.reason = 'tracking wheel'

    @synchronized
    def freeze_for_throw(self, now):
        self.target = self.value
        self.interpolating = False
        self.freeze_until = now+.6

    @synchronized
    def tick(self, now):
        dt = 0. if self.previous is None else max(0., now-self.previous)
        self.previous = now
        target = self.target
        if not self.enabled:
            self.value = 0.
            self.interpolating = False
        elif now-self.valid_at > .2:
            # Missing measurements are not a command to steer straight.
            self.target = self.value
            self.interpolating = False
            if self.reason == 'tracking wheel':
                self.reason = 'holding last steering — waiting for tracking'
        else:
            if self.interpolating:
                fraction = max(0.,min(1.,(now-self.transition_at)/(self.interpolation_ms/1000)))
                target = self.transition_from + (self.target-self.transition_from)*fraction
                if fraction >= 1:
                    self.interpolating = False
            tau = self.smoothing
            self.value += (target-self.value)*(1-math.exp(-dt/tau)) if tau > 0 else target-self.value
        return self.value

    @synchronized
    def status(self):
        return dict(steering_enabled=self.enabled, steering_angle_deg=self.angle,
                    steering_output=round(self.value,3), steering_state=self.reason,
                    steering_height_delta=self.height_delta,
                    steering_curve={0:'Linear',1:'Gentle',2:'Strong'}.get(self.curve,'Linear'),
                    steering_interpolation_ms=self.interpolation_ms,
                    steering_mapping='proportional wrist height — manual strength')


def gamecube_steering(value):
    """Send the full SDL/uinput range, without a centre offset.

    The wheel profile removes the runtime deadzone. Do not presume the guest
    applies GC PADClamp: the old offset capped output at 22272/32767 and never
    reached exact neutral during exponential decay. Native axis quantisation
    is the only rounding here; physical stick passthrough is unaffected.
    """
    return max(-32768, min(32767, round(value*(32768 if value < 0 else 32767))))


class WheelBridge:
    def __init__(self, root):
        self.process = None
        self.error = ''
        self.root = root
        self.last_send = 0.
        if not os.environ.get('MKW_WHEEL_BRIDGE'):
            self.error = 'Restart game to connect wheel controller'
            return
        candidates = []
        for path in Path('/sys/class/input').glob('event*/device/name'):
            if path.read_text().strip() == '8BitDo Ultimate 2 Wireless Controller':
                candidates.append('/dev/input/'+path.parts[-3])
        if len(candidates) != 1:
            self.error = 'Cannot uniquely identify physical source controller'
            return
        try:
            self.process = subprocess.Popen([sys.executable, str(root/'wheel_virtual.py'), candidates[0]],
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
            # Driver publishes READY immediately after creating the virtual pad.
            if not self.process.stdout.readline().startswith('READY '):
                raise OSError('Virtual steering controller did not start')
        except OSError as exc:
            self.error = str(exc)

    def send(self, wheel, now):
        if not self.process or now-self.last_send < 1/120:
            return
        self.last_send = now
        try:
            self.process.stdin.write(f'{int(wheel.enabled)} {gamecube_steering(wheel.value)}\n')
            self.process.stdin.flush()
        except (OSError, ValueError) as exc:
            self.error = str(exc)
            self.close()

    def status(self):
        return dict(steering_bridge_ready=bool(self.process and self.process.poll() is None),
                    steering_error=self.error)

    def close(self):
        if self.process:
            process, self.process = self.process, None
            try:
                process.stdin.close()
                process.wait(timeout=2)
            except (OSError, subprocess.TimeoutExpired):
                process.terminate()
                process.wait(timeout=2)


class WheelOutput:
    """Clock steering independently of the blocking body-inference loop."""
    def __init__(self,wheel,bridge):
        self.wheel,self.bridge=wheel,bridge
        self.stopping=threading.Event()
        self.thread=threading.Thread(target=self.run,name='wheel-output',daemon=True)
        self.thread.start()

    def run(self):
        while not self.stopping.is_set():
            now=time.monotonic()
            with self.wheel.lock:
                self.wheel.tick(now)
                self.bridge.send(self.wheel,now)
            self.stopping.wait(1/120)

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=3)

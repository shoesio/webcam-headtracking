"""Lean hysteresis and an additive, keyboard-only drift button.

No joystick is created. A lost pose, disabled feature or process exit releases
the gesture's key; a real controller's drift button is never modified.
"""
import ctypes
import math
import os
import struct


class LeanGesture:
    def __init__(self):
        self.held = False
        self.filtered = None
        self.previous = None
        self.candidate_since = None

    def update(self, roll, sample_time, now, enabled, activation=12.0,
               release=6.0, confirmation_ms=120.0):
        if not enabled or sample_time <= 0 or now - sample_time > 0.20:
            self.held = False
            self.filtered = None
            self.previous = None
            self.candidate_since = None
            return False
        if self.previous != sample_time:
            dt = 1 / 30 if self.previous is None else max(0, sample_time - self.previous)
            self.filtered = (roll if self.filtered is None else
                             self.filtered + (roll - self.filtered) * (1 - math.exp(-dt / 0.080)))
            self.previous = sample_time
        angle = abs(self.filtered)
        if self.held:
            if angle <= release:
                self.held = False
                self.candidate_since = None
        elif angle >= activation:
            if self.candidate_since is None:
                self.candidate_since = now
            if now - self.candidate_since >= confirmation_ms / 1000:
                self.held = True
        else:
            self.candidate_since = None
        return self.held


class DriftKeyboard:
    def __init__(self):
        self.display = None
        self.held = False

    def open(self):
        if self.display is not None:
            return
        self.x = ctypes.CDLL('libX11.so.6')
        self.xtest = ctypes.CDLL('libXtst.so.6')
        self.x.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x.XOpenDisplay.restype = ctypes.c_void_p
        self.x.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x.XKeysymToKeycode.restype = ctypes.c_uint
        self.x.XFlush.argtypes = [ctypes.c_void_p]
        self.x.XCloseDisplay.argtypes = [ctypes.c_void_p]
        self.xtest.XTestFakeKeyEvent.argtypes = [ctypes.c_void_p, ctypes.c_uint,
                                                ctypes.c_int, ctypes.c_ulong]
        self.display = self.x.XOpenDisplay(os.environ.get('DISPLAY', ':1').encode())
        if not self.display:
            raise OSError('Cannot connect to the game keyboard display')
        self.keycode = self.x.XKeysymToKeycode(self.display, 0xffc5)  # F8
        if not self.keycode:
            self.x.XCloseDisplay(self.display)
            self.display = None
            raise OSError('F8 is not mapped on the game display')

    def set(self, held):
        if held == self.held:
            return
        if held:
            self.open()
        if self.display is not None:
            if not self.xtest.XTestFakeKeyEvent(self.display, self.keycode, int(held), 0):
                raise OSError('Game display rejected gesture key event')
            self.x.XFlush(self.display)
        self.held = held

    def close(self):
        if self.display is not None:
            try:
                self.set(False)
            finally:
                self.x.XCloseDisplay(self.display)
                self.display = None


def write_bindings(path):
    """Generate the verified v3 format; never replace existing bindings."""
    from pathlib import Path
    path = Path(path)
    buttons = [0x100, 0x200, 0x400, 0x800, 0x1000, 0x10,
               0x40, 0x20, 8, 4, 1, 2]
    data = bytearray(struct.pack('<II', 0x444e424b, 3) + bytes(24))
    for port in range(4):
        data += struct.pack('<?', port == 0)
        for button in buttons:
            data += struct.pack('<iH2x', 65 if port == 0 and button == 0x20 else -1, button)
        for axis in range(10):
            data += struct.pack('<iHh', 65 if port == 0 and axis == 9 else -1,
                                axis, 1 if port == 0 and axis == 9 else 0)
    with path.open('xb') as file:
        file.write(data)

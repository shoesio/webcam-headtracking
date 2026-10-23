"""Analogue uinput wheel; mirrors a source pad without grabbing it."""
import fcntl
import glob
import os
import selectors
import struct
import sys
import time

AXES = [0, 1, 3, 4, 2, 5, 16, 17]
KEYS = [304, 305, 307, 308, 310, 311, 314, 315, 316, 317, 318]
EVENT = struct.Struct('llHHi')


class VirtualWheel:
    def __init__(self):
        self.fd = os.open('/dev/uinput', os.O_WRONLY | os.O_NONBLOCK)
        try:
            for ev in (1, 3):
                fcntl.ioctl(self.fd, 0x40045564, ev)
            for key in KEYS:
                fcntl.ioctl(self.fd, 0x40045565, key)
            for axis in AXES:
                fcntl.ioctl(self.fd, 0x40045567, axis)
            maximum = [0]*64
            minimum = [0]*64
            for axis in AXES:
                maximum[axis] = 255 if axis in (2, 5) else 1 if axis in (16, 17) else 32767
                minimum[axis] = 0 if axis in (2, 5) else -1 if axis in (16, 17) else -32768
            device = struct.pack('80sHHHHI', b'MKVR Gesture Wheel', 3, 0x045e, 0x028e, 1, 0)
            device += struct.pack('256i', *(maximum+minimum+[0]*128))
            os.write(self.fd, device)
            fcntl.ioctl(self.fd, 0x5501)
            name = bytearray(128)
            fcntl.ioctl(self.fd, 0x8080552c, name, True)
            self.sysname = name.split(b'\0')[0].decode()
        except BaseException:
            os.close(self.fd)
            raise

    def emit(self, kind, code, value):
        os.write(self.fd, EVENT.pack(0, 0, kind, code, value))

    def close(self):
        fcntl.ioctl(self.fd, 0x5502)
        os.close(self.fd)


def main():
    wheel = VirtualWheel()
    source = None
    try:
        print('READY', wheel.sysname, flush=True)
        if '--probe' in sys.argv:
            for _ in range(30):
                paths = glob.glob('/sys/class/input/'+wheel.sysname+'/event*')
                if paths:
                    path = '/dev/input/'+os.path.basename(paths[0])
                    if os.path.exists(path):
                        print(path, 'readable', os.access(path, os.R_OK), flush=True)
                        if os.access(path, os.R_OK):
                            break
                time.sleep(.1)
            return
        source = os.open(sys.argv[1], os.O_RDONLY | os.O_NONBLOCK)
        trigger = False
        physical_east = False
        for axis in AXES:
            state = bytearray(24)
            fcntl.ioctl(source, 0x80184540+axis, state, True)
            value = struct.unpack('6i', state)[0]
            if axis == 2:
                trigger = value > 32
                value = 0  # LT accelerates; never also hold the item trigger.
            wheel.emit(3, axis, value)
        bits = bytearray(96)
        fcntl.ioctl(source, 0x80604518, bits, True)
        for key in KEYS:
            value = (bits[key//8] >> (key % 8)) & 1
            if key == 305:
                physical_east = bool(value)
                value = int(physical_east or trigger)
            wheel.emit(1, key, value)
        wheel.emit(0, 0, 0)
        selector = selectors.DefaultSelector()
        selector.register(source, selectors.EVENT_READ)
        selector.register(0, selectors.EVENT_READ)
        buffered = b''
        enabled, steering = False, 0
        while True:
            for ready, _ in selector.select(.1):
                if ready.fd == 0:
                    data = os.read(0, 4096)
                    if not data:
                        return
                    buffered += data
                    while b'\n' in buffered:
                        line, buffered = buffered.split(b'\n', 1)
                        en, value = map(int, line.split())
                        enabled = bool(en)
                        steering = max(-32768, min(32767, value))
                        if enabled:
                            wheel.emit(3, 0, steering)
                        else:
                            state = bytearray(24)
                            fcntl.ioctl(source, 0x80184540, state, True)
                            wheel.emit(3, 0, struct.unpack('6i', state)[0])
                        wheel.emit(0, 0, 0)
                else:
                    data = os.read(source, EVENT.size*64)
                    if not data:
                        return
                    for _, _, kind, code, value in EVENT.iter_unpack(data):
                        if kind == 3 and code == 2:
                            trigger = value > 32
                            wheel.emit(1, 305, int(physical_east or trigger))
                            value = 0
                        elif kind == 1 and code == 305:
                            physical_east = bool(value)
                            value = int(physical_east or trigger)
                        if kind == 3 and code == 0 and enabled:
                            value = steering
                        if kind in (0, 1, 3):
                            wheel.emit(kind, code, value)
    finally:
        if source is not None:
            os.close(source)
        wheel.close()


if __name__ == '__main__':
    main()

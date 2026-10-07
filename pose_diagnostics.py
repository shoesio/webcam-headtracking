"""Opt-in, bounded numeric traces; never records camera images."""
import json
import threading
import time
from pathlib import Path


class PoseDiagnostics:
    def __init__(self, root):
        self.root = Path(root)
        self.lock = threading.Lock()
        self.until = 0.0
        self.check_at = 0.0
        self.file = None
        self.count = 0

    def record(self, stage, **values):
        now = time.monotonic()
        with self.lock:
            if now >= self.check_at:
                self.check_at = now + 1.0
                request = self.root / "diagnostics.request"
                if request.exists():
                    request.unlink()
                    if self.file:
                        self.file.close()
                    name = time.strftime("pose-trace-%Y%m%d-%H%M%S.jsonl", time.gmtime())
                    self.file = (self.root / name).open("w", buffering=65536)
                    self.until = now + 120.0
                    self.count = 0
            if self.file and now >= self.until:
                self.file.close()
                self.file = None
            if not self.file:
                return
            self.file.write(json.dumps(dict(t=now, stage=stage, **values)) + "\n")
            self.count += 1
            if self.count % 150 == 0:
                self.file.flush()


diagnostics = PoseDiagnostics(Path.home() / "mkw-headtracker")

"""Bounded, labelled numeric recordings; no images or automatic model changes."""
import json
from pathlib import Path
import time


class TrainingRecorder:
    LABELS = {1: 'overarm', 2: 'underarm', 3: 'normal_movement'}

    def __init__(self, root):
        self.root = Path(root)
        self.request = None
        self.file = None
        self.label = ''
        self.path = ''
        self.started = 0.0
        self.samples = 0
        self.mode = False
        self.last_flush = 0.0

    def update(self, config, now):
        label = int(config.get('training_label', 0))
        request = int(config.get('training_request', 0))
        self.mode = label in self.LABELS
        if self.request != request:
            self.close()
            self.request = request
            if self.mode:
                folder = self.root / 'throw-training'
                folder.mkdir(exist_ok=True)
                self.label = self.LABELS[label]
                self.path = str(folder / (time.strftime('%Y%m%d-%H%M%S',time.gmtime())
                                         + f'-{self.label}-{request}.jsonl'))
                self.file = open(self.path, 'x')
                self.started = now
                self.samples = 0
                self.file.write(json.dumps(dict(type='metadata', label=self.label,
                    started_utc=time.time(), started_monotonic=now, max_seconds=60,
                    coordinate_system='MediaPipe normalized image and world landmarks',
                    note='Whole session label includes wind-up, throw and return; not per-frame ground truth'))+'\n')
        if not self.mode or (self.file and now-self.started >= 60):
            self.close()

    def record(self, points, world, capture_time, result_time):
        if not self.file or capture_time < self.started:
            return
        self.samples += 1
        self.file.write(json.dumps(dict(type='sample', t=capture_time,
            result_time=result_time, elapsed=capture_time-self.started,
            detected=bool(points), landmarks=points, world_landmarks=world))+'\n')
        if result_time-self.last_flush >= 1:
            self.file.flush()
            self.last_flush = result_time

    def close(self):
        if self.file:
            self.file.write(json.dumps(dict(type='end',samples=self.samples,
                                            stopped_monotonic=time.monotonic()))+'\n')
            self.file.close()
            self.file = None

    def status(self, now):
        return dict(training_mode=self.mode, training_recording=self.file is not None,
                    training_label=self.label, training_samples=self.samples,
                    training_elapsed_s=round(min(60,max(0,now-self.started)),1)
                    if self.started else 0, training_file=self.path)

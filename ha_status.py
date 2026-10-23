#!/usr/bin/env python3
import json
import time
import base64
import sys
from pathlib import Path

path = Path.home() / "mkw-headtracker" / "status.json"
try:
    data = json.loads(path.read_text())
except (OSError, ValueError):
    data = {"status": "offline", "quality": 0}

if time.time() - float(data.get("updated_at", 0)) > 4.0:
    data["status"] = "offline"
    data["quality"] = 0

if "--preview" in sys.argv:
    try:
        data["preview_jpeg"] = base64.b64encode(path.with_name("preview.jpg").read_bytes()).decode("ascii")
    except OSError:
        pass
try:
    arms = json.loads(path.with_name('arm_status.json').read_text())
    if time.time()-float(arms.get('updated_at',0)) < 4:
        data.update({key:value for key,value in arms.items()
                     if key not in ('updated_at','arm_landmarks')})
except (OSError,ValueError):
    pass
print(json.dumps(data, separators=(",", ":")))

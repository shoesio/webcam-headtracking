#!/usr/bin/env python3
import json
import time
from pathlib import Path

path = Path.home() / "mkw-headtracker" / "status.json"
try:
    data = json.loads(path.read_text())
except (OSError, ValueError):
    data = {"status": "offline", "quality": 0}

if time.time() - float(data.get("updated_at", 0)) > 4.0:
    data["status"] = "offline"
    data["quality"] = 0

print(json.dumps(data, separators=(",", ":")))


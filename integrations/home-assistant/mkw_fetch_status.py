#!/usr/bin/env python3
"""Fetch tracker health and a private HA camera snapshot over existing SSH."""
import base64
import json
import subprocess
from pathlib import Path

try:
    target_file = Path('/config/mkw_machine.json')
    machine = json.loads(target_file.read_text()).get('machine', 'BC-250') if target_file.exists() else 'BC-250'
    host = {'BC-250': 'mkw-bc250', 'Steam Deck': 'mkw-steamdeck'}[machine]
    result = subprocess.run([
        "ssh", "-F", "/config/mkw_ssh_config", "-o", "BatchMode=yes", "-o", "ConnectTimeout=2",
        host,
        "/home/deck/mkw-headtracker/ha-status", "--preview"
    ], capture_output=True, text=True, timeout=5, check=True)
    status = json.loads(result.stdout)
    status['machine'] = machine
    image = status.pop("preview_jpeg", None)
    if image:
        jpeg = base64.b64decode(image, validate=True)
        if jpeg.startswith(b"\xff\xd8"):
            target = Path("/config/mkw-preview.jpg")
            temporary = target.with_suffix(".tmp.jpg")
            temporary.write_bytes(jpeg)
            temporary.replace(target)
    print(json.dumps(status, separators=(",", ":")))
except (OSError, ValueError, subprocess.SubprocessError):
    print(json.dumps({"status":"offline", "quality":0}))

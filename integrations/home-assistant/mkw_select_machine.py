"""Select only the status/preview destination; game controls use the HA selector."""
import json
import sys
from pathlib import Path

machine = sys.argv[1]
if machine not in ('BC-250', 'Steam Deck'):
    raise ValueError('Unknown Mario Kart machine')
target = Path('/config/mkw_machine.json')
temporary = target.with_suffix('.tmp')
temporary.write_text(json.dumps({'machine': machine}))
temporary.replace(target)
# Never show the previous machine's camera image after a switch.
Path('/config/mkw-preview.jpg').unlink(missing_ok=True)

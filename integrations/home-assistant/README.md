# Home Assistant integration examples

These are isolated examples, not a dump of the household's HA configuration.
No SSH key, token, camera image or real host address is included.

1. Configure key-based SSH from HA to the tracker as your ordinary Linux user.
   Verify each host fingerprint and populate HA's SSH `known_hosts` file.
2. Adapt `ssh_config.example` and save it as `/config/mkw_ssh_config` on HA.
   The sample addresses are documentation placeholders. Replace the hostnames,
   username and key paths for your machines; do not disable host-key checking.
3. Keep the original `ha-control` and `ha-status` wrappers when restoring their
   original bundled runtime. For a new `.venv`, change those wrappers to invoke
   `$HOME/mkw-headtracker/.venv/bin/python` instead.
4. Review `package.example.yaml`, adjust the `/home/deck` remote paths, and merge
   it using your existing HA package convention. Do not replace configuration.yaml.
   Its machine selector routes new writes; changing machines does not copy all
   settings automatically. Apply a value to send current control values.
5. Set all eight helpers deliberately before applying: unset helpers can resolve
   to zero. Starting rotational gains 2/2/1, XYZ 0/0/0, smoothing .5 and prediction
   0 are a conservative head-only baseline. Existing helpers may restore values.

The example controls head pose; gestures also use the same SSH CLI:

```sh
ha-control lean 1 12 6 0
ha-control throws 1 0.25 0.4
ha-control training overarm
ha-control training stop
```

Those commands are run remotely using the absolute wrapper path, not installed
in PATH by this repository. Lean arguments are enable/activation/release/confirm
milliseconds. Throws arguments are enable/stroke/visibility. Training modes:
`overarm`, `underarm`, `normal`, `stop`. Training writes local numeric landmarks,
not images. Recordings should stay off GitHub.

For health/preview polling use the sanitized `mkw_fetch_status.py` example and
`mkw_select_machine.py` on HA. The fetcher prints JSON, writes the latest preview
to `/config/mkw-preview.jpg`, and reports offline when SSH fails. The machine
selection command is:

```sh
python3 /config/mkw_select_machine.py 'BC-250'
```

Or use `'Steam Deck'`. Wire that command to your machine-selector automation
alongside the control-target selection. It is a separate state file, so both
must follow the same selector. Poll with a command-line sensor about every two
seconds, using `status` as its state and `quality`, `pose_age_ms`, `capture_fps`,
`inference_fps` and `face_fps` as JSON attributes.

Expose the JPEG through HA's authenticated Local File camera integration. Do not
put it in `/config/www` or start an unauthenticated camera HTTP server. Preview
rate is for framing/debugging, not video streaming. Never treat detection quality
as a guarantee of low jitter or safe device control.

# Backup scope and recovery

Exported read-only from the BC-250's `~/mkw-headtracker` on 2026-10-23. Earlier
committed baseline: `6a2b1bc`. The source working tree contained later gesture,
preview/status and hand-wheel changes; these are included in the export commit.
`docs/source-sha256.txt` records source hashes for verification. The original
six Git commits are retained, rather than replacing the project's history.

This is not a complete runnable Mario Kart backup. Preserve separately:

- The custom mono/head-tracked TV `WiiCompiled` executable and matching launcher.
- Its local camera/renderer/input patches if they can be recovered.
- Your own game-derived assets, NAND, Mii database, save files and texture packs.
- Python runtime, downloaded MediaPipe models and machine-specific HA secrets.

The previous game-source checkout came from
https://github.com/heurazy/mario-kart-wii-VR-port at revision `0e2472d`. That
checkout did **not** contain the recovered executable's custom local changes.
Do not rebuild that upstream revision over the working binary and expect this
tracker repository to restore those camera changes.

Known recovered custom executable SHA256:
`b998d8fa2f139948a3c36ba8914ae5817e156867c09941b88207ba8456319c32`.
This identifies the historical recovered binary; the binary is not uploaded.

The sender emits a UTF-8 UDP datagram to `127.0.0.1:26760` containing six
space-separated numbers: `yaw pitch roll x y z`. Angles are degrees and
translations originate in the face transformation estimate, multiplied by the
configured gains. They are not guaranteed physical metres. The original game
applies them as a local/additive viewpoint transform. Other games need their
own receiver/adaptor; this is not OpenTrack or OpenXR wire-format compatibility.

The Steam Deck's later HORI pedal helper and arcade-game launchers are separate
integration work, not part of this BC-250 tracker export. In particular, F6/F7
already belong to throw direction; do not reuse them for pedals. The later Deck
fix moved pedal keys to F4/F5. F8 is lean/drift and F9 is item use.

The original repository directory contains an `app/` sibling from pinball work.
That application was not tracked by the original Git history and is outside
this tracker export. No changes were made to the BC-250 repository or game.

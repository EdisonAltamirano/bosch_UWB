# D6 - Rosbag Recording

Date: 2026-07-16

## Status: BLOCKED by a pre-existing environment gap — not caused by new_uwb

`sensors/sensors/uwb_rosbag_recorder_node.py` (reused unmodified by
`new_uwb`, see `new_uwb/launch/new_uwb.launch.py`) hardcodes
`storage_id="mcap"` when opening the bag writer. This container
(`uwb_nxp`) does not have the `mcap` rosbag2 storage plugin installed:

```
dpkg -l | grep rosbag2
  ros-humble-rosbag2-storage-default-plugins   <- SQLite3 only
  (no mcap package)

apt-cache search mcap   -> no results at all
apt-get install -y ros-humble-rosbag2-storage-mcap
  -> E: Unable to locate package ros-humble-rosbag2-storage-mcap
```

`apt-get update` succeeds (container does have internet access — the
package genuinely isn't offered by the configured apt sources for this
image, this isn't a stale-cache issue). `Dockerfile` never installs it
either (checked — only `pip install pyserial tqdm numpy opencv-python
scipy matplotlib rosbags numba` on the Python side; no
`rosbag2-storage-mcap` apt package anywhere).

**This blocks rosbag recording for legacy_tlv too** — it is not specific to
`new_uwb`. `uwb_processing/uwb_processing/loaders.py:214` hardcodes the same
`storage_id="mcap"` on the read side, so even switching the recorder to
`sqlite3` locally wouldn't let `run_session.py` read the result without
also patching the reader — not done here, since patching shared
`sensors`/`uwb_processing` files to route around a missing system package
is out of scope for `new_uwb` and shouldn't happen silently.

**Action needed from the user**: add `ros-humble-rosbag2-storage-mcap` to
the Dockerfile (or confirm it's present in whatever image is actually used
for real recording sessions — this dev container may simply predate that
requirement).

## What was verified instead, without touching rosbag I/O

Recording itself (writer.open() with storage_id="mcap") fails immediately,
so `new_uwb_udp_frame_publisher` was confirmed separately (`D4_ros2_publisher.md`)
to publish correctly-shaped `UwbFrame` messages at the right rate. See
`D6_processing_report.md` for how far the rest of the pipeline was verified
without an actual bag file.

## Gate

**Not evaluated — blocked on infrastructure**, not a new_uwb defect. Not a
board/hardware blocker like D1/D2 either — this one is fixable by installing
one apt package.

# D6 - Processing Report

Date: 2026-07-16

## Second, independent pre-existing environment gap found

Beyond the missing `mcap` plugin (`D6_rosbag_record.md`), importing
`uwb_processing` itself currently fails in this container:

```
uwb_processing/__init__.py -> from .run_session import process_session
  -> run_session.py -> from .breathing import run_breathing_extraction
  -> breathing.py -> from scipy.signal import lfilter
  -> AttributeError: _ARRAY_API not found / ImportError: numpy.core.multiarray failed to import
```

The system `scipy` (`/usr/lib/python3/dist-packages/scipy`, apt-installed)
was built against NumPy 1.x; the environment's actual NumPy is 2.2.6
(installed via the Dockerfile's unpinned `pip install ... numpy ...`).
This is a NumPy 1.x/2.x ABI break, **unrelated to new_uwb or to the mcap
gap** — it would block `python -m uwb_processing.run_session` for
legacy_tlv sessions too, in this container, right now. Also blocked
building the `uwb_processing` ROS2 package itself with colcon:
```
colcon build --packages-select uwb_processing
  -> error: error in 'egg_base' option: 'uwb_processing' does not exist or is not a directory
```
(unexplored further — out of scope; flagging for the user, since it means
`uwb_processing` was apparently never colcon-built in this particular
container, only run via `python -m` directly against source, which sidesteps
the packaging step but not the scipy/numpy issue.)

## What was verified instead: the loaders.py compatibility, directly

`uwb_processing/uwb_processing/loaders.py` and `types.py` do not import
`scipy` — only `numpy`. Loaded them directly by file path (bypassing the
broken `uwb_processing/__init__.py` import chain) and replicated
`_load_rosbag_directory()`'s exact per-message loop body (same dict shape,
same field names) against synthetic `new_uwb` frames — same reassembly +
`parse_cir_udp_payload` path already proven in `D4_ros2_publisher.md`, fed
straight into `loaders._normalize_batched_frame_records` and
`loaders._stack_frame_records` with **zero modification to
`uwb_processing` source**.

Result, 5 synthetic frames (8 samples x 120 taps each):

```
RadarSession OK: (5, 8, 120) rosbag {'frame_count': 5, 'path_count': 8, 'tap_count': 120}
timestamps_s: [0.  0.1 0.2 0.3 0.4]
```

Shape `(frames, paths, taps) = (5, 8, 120)` matches the synthetic input
exactly, and timestamps came out monotonically ordered and correctly
offset. This is the load-bearing part of D6 as far as `new_uwb` is
concerned: it proves `loaders.py`'s bag-message-to-`RadarSession` logic
accepts `new_uwb`-shaped data with **no protocol-specific branch added** —
addendum's explicit D6 requirement. Everything downstream of
`RadarSession` (`preprocessing.py`, `detection.py`, `plotting.py`) is
already protocol-agnostic and exercised daily by legacy_tlv sessions, so it
was not re-tested here.

## What remains genuinely unverified

1. The literal `rosbag2_py` write/read round trip (blocked by the mcap gap).
2. `run_session.py`'s CLI/plotting/detection path end-to-end in this
   container (blocked by the scipy/numpy ABI break — independent of
   `new_uwb`).
3. Everything that requires real hardware data rather than synthetic
   payloads (D1/D2 still pending).

## Gate

**Partially passed, by direct code-path verification rather than the
literal CLI run the addendum specified** — the addendum's actual concern
("does `uwb_processing` need a protocol branch to handle `new_uwb` data")
has a verified **no** answer. The literal `uwb_processing.run_session`
CLI invocation is blocked by two pre-existing, unrelated environment
issues (mcap plugin, scipy/numpy ABI) that affect legacy_tlv equally and
are outside `new_uwb`'s scope to fix silently.

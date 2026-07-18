# D4 - ROS2 Publisher Node

Date: 2026-07-16

## What was built

- `new_uwb/` — new ROS2 `ament_python` package, separate from `sensors`
  (addendum Decision C), built successfully with `colcon build --packages-select new_uwb`
  inside the `uwb_nxp` container.
- `new_uwb_udp_frame_publisher` node: owns the single UDP socket on
  `listen_port` (default 20000), decodes all four `CMD_*` packet kinds via
  `new_uwb.protocol`, reassembles `CMD_CIR_REPORT` fragment trains, and
  publishes:
  - `UwbFrame` (reused unmodified from `sensors_interfaces`) on
    `/uwb/frame_raw`, using `sensors.sr250_protocol.parse_cir_udp_payload`
    for the inner CIR metadata — same cross-package import pattern already
    used by `uwb_processing/loaders.py`.
  - `std_msgs/String` (JSON) on `/uwb/new_uwb_control_status` for
    ACK/ERROR/SYSTEM_STATE, so the control node doesn't need its own
    receive socket on the board's data port (see D5).
- `new_uwb_test_sender` node: synthetic `CMD_CIR_REPORT` generator (same
  raw SR250 payload shape as `sensors/sensors/uwb_test_sender.py`'s
  hardware-free test, but fragmented the way `uwb_sw`'s `nucleo_udp_send()`
  actually does it) — built specifically to validate D4-D6 without a
  flashed board.

## Validation performed (synthetic, no hardware — real board run still pending)

Ran inside the `uwb_nxp` container: `new_uwb_udp_frame_publisher` +
`new_uwb_test_sender` (8 samples x 120 taps, 20 Hz) over loopback.

```
ros2 topic list
  /uwb/frame_raw
  /uwb/new_uwb_control_status

ros2 topic hz /uwb/frame_raw   -> average rate: 20.00, low jitter (std dev ~0.006s)

ros2 topic echo /uwb/frame_raw --once:
  msg_type: 7
  radar_data_type: 0
  session_handle: 287454020   # 0x11223344, matches the synthetic sender's fixed value
  status: 0
  num_samples: 8
  block_size: 512
  bytes_per_tap: 4
  raw_payload: [...]
```

`num_samples=8`, `block_size=512`, `bytes_per_tap=4` are exactly what the
sender's parameters (`num_samples=8`, `taps_per_block=120` ->
`taps_per_sample=128` -> `cir_data_bytes_per_block=120*4=480` ->
`block_size=32+480=512`) predict. This confirms, end to end:

1. `CMD_CIR_REPORT` fragments (4 fragments per 4106-byte frame at this size,
   matching `MAX_DATA_PER_ETH_PACKET=1184` chunking) are received and
   reassembled correctly.
2. The reassembled bytes are valid input to
   `sensors.sr250_protocol.parse_cir_udp_payload` — the cross-firmware raw
   NTF payload compatibility argued in `D3_parser_report.md` holds in
   practice, at least for synthetic data shaped like the real thing.
3. Sustained 664 datagrams / 166 frames over one run, `malformed=0`.

Publisher's own status log during the run:
```
status: datagrams=664 ack=0 error=0 system_state=0 cir_fragments=664 cir_frames=166 malformed=0 reassembly_buffer_bytes=0 last_datagram_age=0.01s
```

## What this does NOT prove

This is synthetic data built from the same generator logic as legacy_tlv's
existing hardware-free test, not a real hardware capture. The open question
from D3 (does `uwb_sw`'s real SR250 NTF payload actually match this shape
byte-for-byte) is still unverified — that needs a flashed board (D1/D2).

## Gate

**Passed (synthetic)**: `/uwb/frame_raw` produces
`sensors_interfaces/msg/UwbFrame` at the expected rate with correctly
decoded fields. Real-hardware confirmation still pending (blocked on D1/D2).

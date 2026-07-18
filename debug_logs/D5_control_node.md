# D5 - ROS2 Control Node

Date: 2026-07-16

## Design decision: single socket owner (per addendum's D5 warning)

`new_uwb_node` never binds a receive socket — it only `sendto()`s to the
board's command port. All inbound traffic (ACK/ERROR/SYSTEM_STATE/CIR) stays
owned by `new_uwb_udp_frame_publisher`, which republishes ACK/ERROR/
SYSTEM_STATE as JSON on `/uwb/new_uwb_control_status`. `new_uwb_node`
subscribes to that topic for feedback. This avoids the two-processes-on-
one-UDP-port ambiguity the addendum flagged, without needing
`SO_REUSEADDR` tricks.

## Validation performed (synthetic — loopback, no hardware)

Inside the `uwb_nxp` container: a throwaway "fake board" script bound
`0.0.0.0:37249` to capture whatever `new_uwb_node` actually sends, while
`new_uwb_udp_frame_publisher` + `new_uwb_node` (`board_ip=127.0.0.1`,
`board_port=37249`, `setting_idx=0`, `duration_ms=5000`, `auto_start=true`)
ran normally. Separately, a script sent synthetic ACK/ERROR/SYSTEM_STATE
packets straight to the publisher's port 20000 to exercise the status-topic
feedback path.

**What the fake board actually received:**
```
01 00 88 13 00 00
```
Decoded: `cmd_id=0x01 (CMD_START_SESSION)`, `setting_idx=0x00`,
`duration_ms=0x00001388` little-endian = **5000** — exactly the
`duration_ms=5000` parameter passed in. Byte-exact match with
`new_uwb.protocol.build_start_session_packet`'s documented layout.

**`new_uwb_node` log:**
```
Sent CMD_START_SESSION setting_idx=0 duration_ms=5000 raw=01 00 88 13 00 00
ACK recv_cmd_id=0x01
ERROR ERR_UNKNOWN_USER_CMD_RECEIVED (0x0F)
SYSTEM_STATE SYSTEM_STATE_OK
SYSTEM_STATE SYSTEM_STATE_RADAR_SESSION_DONE
```

All four injected status events round-tripped through
`new_uwb_udp_frame_publisher` -> `/uwb/new_uwb_control_status` ->
`new_uwb_node` correctly, with the right log level (ERROR for the error
event, INFO for the rest) and correct name lookups
(`ERR_UNKNOWN_USER_CMD_RECEIVED`, `SYSTEM_STATE_OK`,
`SYSTEM_STATE_RADAR_SESSION_DONE`).

## Gate

**Passed (synthetic)**:
- Launch sends the start packet — confirmed byte-exact.
- ACK/ERROR/SYSTEM_STATE visible in logs — confirmed, via the status topic.
- CIR frames still reach `/uwb/frame_raw` — confirmed in D4 (same publisher
  process handles both concerns concurrently without interference).

Real-hardware confirmation still pending (blocked on D1/D2, same as D4).

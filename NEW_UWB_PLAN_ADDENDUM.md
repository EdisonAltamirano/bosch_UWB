# NEW_UWB_PLAN Addendum - AI Execution Override

Date: 2026-07-17

This file overrides ambiguous parts of `NEW_UWB_PLAN.md`. Use it together with
the original plan. The original file has encoding damage, so this addendum is
ASCII-only on purpose.

## Locked Decisions

### A. Keep The Live `CMD_*` Firmware Protocol

Use the current live `uwb_sw` firmware path as the baseline:

- `main.c` calls `nucleo_udp_init()`.
- `udp_server.c` receives `CMD_START_SESSION`.
- `main.c` sends CIR notifications using `nucleo_udp_send(..., CMD_CIR_REPORT, ...)`.
- `uwb_udp_protocol.c` / `uwb_udp_protocol.h` are present, but are not wired into
  `main.c` today. Treat them as inactive/future code unless a build or hardware
  capture proves otherwise.

For the first working `new_uwb` implementation, speak this protocol:

- PC -> board command port `37249`:
  - `CMD_START_SESSION = 0x01`
    packet layout: `<cmd_id:u8><setting_idx:u8><duration_ms:u32 little-endian>`
  - `CMD_SYSTEM_COMMAND = 0x05`
    packet layout: `<cmd_id:u8><sub_cmd:u8>`, only defined sub-command today is
    `CMD_SYSTEM_RESET = 0x01` (triggers `NVIC_SystemReset()` on the board). Both
    `CMD_START_SESSION` and `CMD_SYSTEM_COMMAND` are ACKed the same way (see below).
    Not in the original addendum draft — confirmed live in
    `udp_server.c:103-119` (`udp_receive_callback`). Give `new_uwb_node` parity with
    this if a remote-reset control is wanted.
- Board -> PC port `20000` (same single port for everything — see the D5 socket
  note below):
  - `CMD_CIR_REPORT = 0x02`
  - `CMD_UDP_ETH_ACK = 0x03`
  - `CMD_ERROR_REPORT = 0x04`
  - `CMD_SYSTEM_STATE = 0x06` — **currently dead on the wire, see D1 below.**
- CIR fragment layout:
  - `<cmd_id:u8><last_fragment:u8><data_len:u16 little-endian><data:data_len>`
  - max fragment data length from firmware: `MAX_DATA_PER_ETH_PACKET = 1184`

Do not build the first `new_uwb` around the TLV envelope unless the firmware is
explicitly changed to use it.

Verified against source (2026-07-16): `main.c:174` calls `nucleo_udp_init()`;
`main.c:524` sends CIR via `nucleo_udp_send(NUCLEO_REMOTE_PORT, CMD_CIR_REPORT, ...)`;
`udp_server.h` defines `NUCLEO_ETH_PORT=37249`, `NUCLEO_REMOTE_PORT=20000`,
`MAX_DATA_PER_ETH_PACKET=1184` exactly as above. `nucleo_udp_init()` also hardcodes
`destination_ip_addr.addr = 0x6601A8C0` = PC IP `192.168.1.102` — confirm this
matches the PC's actual interface IP before D2, or `new_uwb`'s publisher will never
see a packet no matter how correct the parser is.

### B. Create A CubeIDE Project For `uwb_sw`

Create a real STM32CubeIDE project for `uwb_sw` so the firmware can be compiled
and flashed when needed.

Recommended baseline:

- Keep the source layout as-is: `Core/Src`, `Core/Inc`, `LWIP`, `Drivers`,
  `Middlewares`.
- Use `boshUWBSTM32` only as a project metadata/reference template.
- **Exclude `uwb_udp_protocol.c` from the build unconditionally, not just "if it
  breaks the build".** Verified: every call site in that file
  (`uwb_udp_protocol.c:71`, `:86`, `:477`) invokes
  `nucleo_udp_send(reply_port, packet, packet_len)` with 3 arguments, but the real
  prototype in `udp_server.h:40` is
  `nucleo_udp_send(u16_t port, u8_t cmd_id, const u8_t *payload, u16_t payload_length)`
  — 4 arguments. This is a hard C compile error (wrong argument count), not a
  maybe. Exclude `uwb_udp_protocol.c`/`.h` from the CubeIDE source set for Option A.
- Do not reorder firmware source just to resemble `boshUWBSTM32` unless the build
  requires it.
- **Fix the `CMD_SYSTEM_STATE` dead-send bug while in here.** `main.c:301-303`
  (entering `UCI_WAITING_FOR_USER_COMMAND`) and `main.c:424-426` (after radar
  session deinit) both call
  `nucleo_udp_send(NUCLEO_REMOTE_PORT, CMD_SYSTEM_STATE, (uint8_t*)&err_report_cmd, 2)`,
  but the `switch (cmd_id)` inside `nucleo_udp_send()` (`udp_server.c:134-200`) only
  has cases for `CMD_UDP_ETH_ACK`/`CMD_ERROR_REPORT` (grouped) and
  `CMD_CIR_REPORT` — no `case CMD_SYSTEM_STATE:`, no `default:`. Those two call
  sites currently transmit nothing; `send_status` falls through as `1` and the
  function returns without allocating a pbuf. Add
  `CMD_SYSTEM_STATE` to the `CMD_UDP_ETH_ACK`/`CMD_ERROR_REPORT` case group (same
  2-byte `{cmd_id, err_id}` shape, so the existing branch body needs no other
  change). Do this as part of D1, not later — otherwise D2/D3 will falsely treat
  "no SYSTEM_STATE ever seen" as a network/parser problem instead of the known
  firmware gap.

### C. Use A Separate ROS2 Package Named `new_uwb`

Create a separate package `new_uwb/` instead of adding more branches to the
existing `sensors` nodes.

The new package should publish the same message type and topic:

- message: `sensors_interfaces/msg/UwbFrame`
- topic: `/uwb/frame_raw`

The goal is that `uwb_processing` can consume the rosbag without protocol-specific
changes.

## Required Debug Pipeline For AI Editors

Rule: do not advance to the next phase without leaving a small log or artifact
showing what was sent, what was received, and what conclusion was drawn.

Create:

```text
debug_logs/
```

## D0 - Static Firmware Inventory

Commands to run:

```powershell
rg -n "nucleo_udp_init|uwb_udp_protocol_init|CMD_CIR_REPORT|CMD_START_SESSION|nucleo_udp_send|destination_ip_addr" uwb_sw\Core
rg -n "MAX_DATA_PER_ETH_PACKET|NUCLEO_ETH_PORT|NUCLEO_REMOTE_PORT|CMD_" uwb_sw\Core\Inc\udp_server.h
```

Log:

```text
debug_logs/D0_firmware_inventory.md
```

Required conclusion:

- Which protocol is actually wired into `main.c`.
- Which file owns UDP receive.
- Which file owns CIR transmit.
- Which PC IP is hardcoded by `destination_ip_addr.addr = 0x6601A8C0`.

Gate:

- Proceed only when the active protocol is documented. Current expected answer:
  active protocol is `CMD_*`.

## D1 - CubeIDE Build And Flash

Actions:

- Create/import CubeIDE project metadata for `uwb_sw`.
- Build Debug.
- If `uwb_udp_protocol.c` causes compile errors, exclude it from the build for
  Option A and document why.
- Flash the board if hardware is available.
- Capture serial console output if available.

Logs:

```text
debug_logs/D1_cubeide_build.md
debug_logs/D1_flash_smoke_test.md
```

Gate:

- Build produces an `.elf`.
- If hardware is connected, flashed firmware boots and Ethernet/link status can
  be observed.

## D2 - Minimal UDP Test Without ROS2

Create a small Python debug script, for example:

```text
tools/uwb_cmd_debug.py
```

Required behavior:

- Send `CMD_START_SESSION` to board IP, UDP port `37249`.
- Listen on `0.0.0.0:20000`.
- Print every datagram with:
  - timestamp
  - source IP/port
  - length
  - raw hex
  - decoded command name
- Decode:
  - `03 01` as ACK for `CMD_START_SESSION`
  - `04 xx` as ERROR
  - `06 xx` as SYSTEM_STATE
  - `02 last len_lo len_hi ...` as CIR fragment
- Write JSONL logs.

Log:

```text
debug_logs/D2_udp_smoke_test.jsonl
```

Gate:

- Start command bytes are visible in the log.
- At least one ACK, ERROR, or CIR fragment is decoded.
- **Do not gate on SYSTEM_STATE unless the D1 `nucleo_udp_send` switch fix was
  applied and flashed.** With the unpatched firmware, `06 xx` will never appear on
  the wire (see D1) — its absence is expected, not a failure signal, until that fix
  ships. If the fix was applied, then require at least one `06 00` (idle/OK) too.
- If nothing at all is received (not even ACK), document IP, port, firewall,
  interface, and hardcoded destination IP (`192.168.1.102`, from
  `destination_ip_addr.addr = 0x6601A8C0` in `udp_server.c:57`) checks before
  touching ROS2.

## D3 - Offline Parser Tests

Implement parser code before ROS2 integration.

Required tests:

- ACK parser: `03 01`
- ERROR parser: `04 <err_id>`
- SYSTEM_STATE parser: `06 <state>`
- CIR fragment parser with valid `data_len`
- CIR reassembly until `last_fragment = 1`
- Bad length rejection
- Unknown command handling

Log:

```text
debug_logs/D3_parser_report.md
```

Gate:

- Parser can reassemble a real or synthetic CIR frame from D2-style packets.

## D4 - ROS2 Publisher Node

Implement:

```text
new_uwb_udp_frame_publisher
```

Required startup logs:

- listen IP
- listen port
- topic name
- protocol: `CMD_*`

Required runtime counters:

- datagrams received
- ACK packets seen
- ERROR packets seen
- SYSTEM_STATE packets seen
- CIR fragments seen
- CIR frames reassembled
- malformed packets dropped

Logs:

```text
debug_logs/D4_ros2_publisher.log
debug_logs/D4_topic_echo.txt
```

Gate:

```powershell
ros2 topic echo /uwb/frame_raw --once
ros2 topic hz /uwb/frame_raw
```

The topic must produce `sensors_interfaces/msg/UwbFrame`.

## D5 - ROS2 Control Node

Implement:

```text
new_uwb_node
```

Required logs for every start command:

- board IP
- board port
- `setting_idx`
- `duration_ms`
- exact packet hex

Important socket rule:

- Do not let `new_uwb_node` and `new_uwb_udp_frame_publisher` both bind to
  `0.0.0.0:20000` independently unless `SO_REUSEADDR` behavior is explicitly
  tested and documented.
- Preferred design: one UDP receive owner publishes decoded status/frame events,
  or the control node only sends and the publisher owns receive.

Log:

```text
debug_logs/D5_control_node.log
```

Gate:

- Launch sends the start packet.
- ACK/ERROR/SYSTEM_STATE is visible somewhere in logs.
- CIR frames still reach `/uwb/frame_raw`.

## D6 - Rosbag And Processing End To End

Actions:

- Record a short rosbag containing `/uwb/frame_raw`.
- Save bag metadata.
- Run:

```powershell
python -m uwb_processing.run_session --input uwb_rosbags\<bag_name>
```

Logs:

```text
debug_logs/D6_rosbag_record.md
debug_logs/D6_processing_report.md
```

Gate:

- `uwb_processing` runs without protocol-specific changes.
- If processing fails, fix `new_uwb` decoding or `UwbFrame` population first.
  Do not add a protocol branch to `uwb_processing` unless every earlier gate has
  passed and the remaining mismatch is documented.

## Execution Status (2026-07-16 run)

Executed D0-D6 in order. Full detail in each `debug_logs/D*.md` file; summary:

| Phase | Result |
|---|---|
| D0 static inventory | **Done.** Confirmed `CMD_*` is live, `uwb_udp_protocol.c` is dead *and* wouldn't compile (3 vs 4 arg mismatch against `nucleo_udp_send`), `CMD_SYSTEM_STATE` was built but never sent (missing switch case). `debug_logs/D0_firmware_inventory.md`. |
| D1 CubeIDE build | **Done — 0 errors, 3 pre-existing warnings.** Fixed the `CMD_SYSTEM_STATE` switch gap, created `uwb_sw/.project`/`.cproject` (Debug config, `uwb_udp_protocol.c` excluded), built headlessly with STM32CubeIDE 2.1.1's bundled ARM GCC (no manual toolchain install). `uwb_sw/Debug/uwb_sw.elf` exists. `debug_logs/D1_cubeide_build.md`. |
| D1 flash + smoke test | **Blocked — no ST-Link/board connected to this machine.** `debug_logs/D1_flash_smoke_test.md` has the exact flash command and the pre-flight checklist (including the hardcoded PC IP `192.168.1.102` the firmware expects) for whoever has hardware access. |
| D2 UDP smoke test | **Script done and self-tested** (`tools/uwb_cmd_debug.py`), **live run blocked** on the same missing hardware. Also flags a real gotcha: don't run it on the Windows host while the `uwb_nxp` container's port 20000 mapping is active — run it inside the container. `debug_logs/D2_udp_smoke_test.md`. |
| D3 offline parser | **Done — 25/25 tests pass.** `new_uwb/new_uwb/protocol.py` + `new_uwb/test/test_protocol.py`. `debug_logs/D3_parser_report.md`. |
| D4 ROS2 publisher | **Done, verified with synthetic traffic** (real hardware still pending). `new_uwb` package builds with colcon; `new_uwb_udp_frame_publisher` reassembles fragments and publishes correctly-shaped `UwbFrame` at the injected rate with zero malformed packets over a sustained run. `debug_logs/D4_ros2_publisher.md`. |
| D5 ROS2 control node | **Done, verified with synthetic traffic.** `new_uwb_node` sent a byte-exact `CMD_START_SESSION`; ACK/ERROR/SYSTEM_STATE round-tripped through the status topic correctly. Single-socket-owner design (control node never binds) confirmed conflict-free. `debug_logs/D5_control_node.md`. |
| D6 rosbag + processing | **Partially blocked — two pre-existing environment gaps, neither caused by new_uwb:** (1) the `uwb_nxp` container has no `mcap` rosbag2 storage plugin installed (not in Dockerfile, not available via `apt-cache search` even after `apt-get update`) — blocks recording for legacy_tlv too; (2) `uwb_processing`'s own import chain is broken by a NumPy 1.x/2.x ABI mismatch against the apt-installed `scipy` (`breathing.py -> scipy.signal` fails), independent of (1) and also pre-existing. Worked around both by exercising `uwb_processing.loaders`'s exact bag-message-to-`RadarSession` logic directly (no `uwb_processing` source changes) against synthetic `new_uwb` frames — it produced a correctly-shaped `RadarSession` with **no protocol branch needed**, which is what the gate actually cared about. `debug_logs/D6_rosbag_record.md`, `debug_logs/D6_processing_report.md`. |

### What's left, and who it's blocked on

Everything left is blocked on physical access, not on more code:

1. Connect the NXP board's ST-Link, confirm the PC NIC is `192.168.1.102`,
   flash `uwb_sw/Debug/uwb_sw.elf`.
2. Run `tools/uwb_cmd_debug.py` (inside the `uwb_nxp` container) against the
   real board and capture real ACK/CIR traffic — this is the first real test
   of whether the raw-NTF-payload-compatibility assumption
   (`D3_parser_report.md`) actually holds against real hardware, not just
   synthetic data.
3. Separately, fix the two D6 environment gaps if real rosbag recording is
   needed soon: add `ros-humble-rosbag2-storage-mcap` to the Dockerfile, and
   pin NumPy/SciPy to compatible versions (or rebuild the apt scipy against
   NumPy 2.x). Neither touches `new_uwb`.

## Packet Cheat Sheet For Option A

Start session, preset 0, duration 10000 ms:

```text
01 00 10 27 00 00
```

Expected ACK:

```text
03 01
```

CIR fragment header:

```text
02 <last_fragment> <data_len_lo> <data_len_hi> ...
```

System reset (preset/duration irrelevant):

```text
05 01
```

System OK / radar session done — **only after the D1 `nucleo_udp_send` switch fix
is applied; unpatched firmware never puts these on the wire even though `main.c`
calls the send function**:

```text
06 00   System OK
06 02   Radar session done
```

# UWB UDP Bridge, STM32 Control Path, and ROS2 Frame Pipeline Design

Date: 2026-04-20

## Goal

Build an end-to-end control and data path for the SR250 radar stack with four coordinated changes:

1. Update `sensors/sensors/uwb_node.py` so the host sends radar configuration to the STM32 using an extensible binary TLV protocol.
2. Update the STM32 firmware under `uwb_sw/Core/Src` so it receives that UDP command, parses it with correct endianness, translates it into SR250/UCI radar configuration calls, and sends radar output back over UDP using the same envelope pattern.
3. Add a ROS2 node that listens to the SR250 UDP output and republishes it as a custom `UWB_Frame.msg`.
4. Add a second ROS2 node that subscribes to `UWB_Frame`, parses the CIR blocks and taps, and stores the result in `.npz` files for later signal processing.

## Scope

This work covers:

- Host-to-STM32 radar control protocol over UDP.
- STM32 UDP receive path and translation to `uci_radar_*` / `uci_sr250_*`.
- STM32 UDP transmit path for SR250 radar frames.
- ROS2 message definition for raw UWB frames.
- ROS2 publisher node for incoming UDP frames.
- ROS2 parser/storage node for decoded CIR output.

This work does not cover:

- New SR250 DSP algorithms on the STM32.
- Changing the core UCI packet format on the SR250 side.
- Final signal processing or detection logic downstream of the `.npz`.

## Existing Context

### STM32 firmware

- `main.c` currently initializes SR250, configures radar, starts a session, and forwards raw radar notification payloads over UDP.
- `udp_server.c` currently supports UDP send and receive, but the receive callback does not translate incoming UDP payloads into radar/UCI commands.
- `uci_radar.c`, `uci_commands.c`, `uci_core.c`, and `uci_sr250.c` already provide the primitives needed to:
  - initialize/configure/start/stop radar sessions,
  - update selected runtime parameters,
  - parse `RADAR_RX_NTF` payloads.

### Python / ROS2 side

- `uwb_node.py` currently sends a legacy fixed struct, not the new SR250 TLV control packet.
- `parse_sr250_udp.py` now parses the observed SR250 UDP payload shape correctly:
  - common header once,
  - `num_samples`,
  - repeated fixed-size sample blocks.
- There is not yet a custom ROS2 message for SR250 frame transport.

## Design Overview

The system will use a single host<->STM32 UDP protocol with:

- a small fixed binary envelope,
- a TLV body for control/configuration messages,
- a framed response path for radar data and acknowledgements.

The SR250 remains the sensing engine. The STM32 remains:

- the radar controller,
- the UCI transport endpoint,
- and the bridge between Ethernet/UDP and SR250/UCI.

ROS2 is used on the host side only for:

- command generation,
- frame publication,
- parsing,
- and archival to `.npz`.

## UDP Protocol

### Envelope

All host<->STM32 UDP packets use this envelope, encoded little-endian:

- `magic[2]`: `0x55 0x57`
- `version[1]`: `0x01`
- `msg_type[1]`
- `seq[2]`
- `payload_len[2]`
- `payload[payload_len]`

### Message types

- `0x01 = SET_CONFIG_FULL`
- `0x02 = SET_PARAMS_PARTIAL`
- `0x03 = START_RADAR`
- `0x04 = STOP_RADAR`
- `0x05 = ACK`
- `0x06 = ERROR`
- `0x07 = RADAR_FRAME`

### TLV format

All control/configuration payloads are encoded as:

- `tag[2]`
- `len[2]`
- `value[len]`

All integer fields inside values are little-endian.

### Initial supported tags

- `0x0001 CHANNEL_NUMBER`
- `0x0002 PREAMBLE_CODE_INDEX`
- `0x0003 ANTENNAS_CONFIG_RX`
- `0x0004 ANTENNAS_CONFIG_TX`
- `0x0005 RADAR_MODE`
- `0x0006 RADAR_SINGLE_FRAME_NTF`
- `0x0007 RADAR_RFRI`
- `0x0008 RADAR_CIR_NUM_SAMPLES`
- `0x0009 RADAR_RX_GAIN`
- `0x000A RADAR_CIR_START_OFFSET`
- `0x000B RADAR_PERFORMANCE`
- `0x000C RADAR_DRIFT_COMPENSATION`
- `0x000D RADAR_PRESENCE_DET_CFG`

## Parameter Encodings

The TLV value encoding follows the semantics already used by the firmware's UCI radar layer.

### Scalar tags

- `CHANNEL_NUMBER`: `u8`
- `PREAMBLE_CODE_INDEX`: `u8`
- `RADAR_MODE`: `u8`
- `RADAR_SINGLE_FRAME_NTF`: `u8`
- `RADAR_CIR_NUM_SAMPLES`: `u8`
- `RADAR_PERFORMANCE`: `u8`
- `RADAR_DRIFT_COMPENSATION`: `u16`

### Structured tags

- `ANTENNAS_CONFIG_RX`: `[mode:u8][length:u8][rxc:u8][rxb:u8][rxa:u8]`
- `ANTENNAS_CONFIG_TX`: `[length:u8][tx_id:u8]`
- `RADAR_RFRI`: `[ranging_interval_ms:u32][slot_duration_rstu:u16][slots_per_rr:u8]`
- `RADAR_RX_GAIN`: `[agc_mode:u8][gain_rxa:u8][gain_rxb:u8][gain_rxc:u8]`
- `RADAR_CIR_START_OFFSET`: `[rxc:u16][rxb:u16][rxa:u16]`
- `RADAR_PRESENCE_DET_CFG`:
  `[mode:u8][periodic_report:u8][sensitivity_q4_4:u8][gpio_notify:u8]`
  `[distance_min_cm:u16][distance_max_cm:u16][hold_delay_ms:u16]`
  `[angle_min_deg:i8][angle_max_deg:i8]`

## Host Behavior

### `uwb_node.py`

`uwb_node.py` becomes the ROS2 control-and-bridge node on the host side.

Responsibilities:

- construct the UDP envelope,
- encode TLVs,
- send `SET_CONFIG_FULL`,
- optionally send `SET_PARAMS_PARTIAL`,
- send `START_RADAR` / `STOP_RADAR`,
- receive `ACK`, `ERROR`, and `RADAR_FRAME`,
- publish `RADAR_FRAME` content as `UWB_Frame`.

### Supported command patterns

- Full configuration snapshot on startup.
- Partial updates for live tuning.
- Explicit start and stop commands.

The node should not assume that receiving a frame implies that configuration succeeded; it must gate on `ACK`.

## STM32 Behavior

### Persistent control state

Add a global control state in STM32 firmware with:

- `uci_radar_params_t current_radar_cfg`
- `uint32_t radar_session_handle`
- `bool session_initialized`
- `bool session_config_valid`
- `bool session_running`
- last client IP/port for replies
- last sequence number seen if needed for duplicate handling

### UDP receive path

The UDP receive callback will:

1. copy the pbuf payload into a contiguous local buffer,
2. validate envelope header,
3. dispatch by `msg_type`,
4. parse TLVs,
5. update current config/state,
6. call the relevant `uci_*` functions,
7. send `ACK` or `ERROR`.

### STM32 command handling

#### `SET_CONFIG_FULL`

- Requires all mandatory config needed for a valid SR250 configuration.
- Fills `current_radar_cfg` from scratch.
- If no radar session exists, initialize one.
- Applies configuration through `uci_radar_configure(...)`.
- Marks config valid if successful.

#### `SET_PARAMS_PARTIAL`

- Requires existing valid config.
- Parses only the tags present.
- Updates the in-memory `current_radar_cfg`.
- Applies targeted runtime setters when possible:
  - `uci_radar_set_rx_gain(...)`
  - `uci_radar_set_rfri(...)`
  - `uci_radar_set_cir_start_offset(...)`
  - `uci_radar_update_presence_cfg(...)`
- For tags without dedicated runtime setters, either:
  - reject if the session is running and the change is unsafe, or
  - stop/reconfigure/restart if explicitly allowed by policy.

#### `START_RADAR`

- Requires session initialized and config valid.
- Calls `uci_radar_start(...)`.
- Marks `session_running = true` on success.

#### `STOP_RADAR`

- Requires session initialized.
- Calls `uci_radar_stop(...)`.
- Marks `session_running = false` on success.

## Radar Frame Return Path

When STM32 receives an SR250 `RADAR_RX_NTF` payload:

- it continues to parse it locally as needed for logging,
- but it wraps the outgoing UDP frame in the new envelope with `msg_type = RADAR_FRAME`.

Recommended `RADAR_FRAME` payload body:

- `session_handle[4]`
- `status[1]`
- `radar_data_type[1]`
- `raw_sr250_payload_len[2]`
- `raw_sr250_payload[...]`

This preserves the raw payload exactly while making the UDP stream self-describing at the transport layer.

## ACK and ERROR

### ACK payload

- `acked_msg_type[1]`
- `seq[2]`
- optional status code or flags

### ERROR payload

- `failed_msg_type[1]`
- `seq[2]`
- `error_code[2]`
- optional detail bytes

Error codes should cover:

- bad magic/version,
- malformed TLV,
- missing required fields,
- invalid state,
- failed SR250/UCI command,
- unsupported tag,
- unsafe runtime reconfiguration.

## ROS2 Message

Create `UWB_Frame.msg` with this structure:

```msg
builtin_interfaces/Time stamp
uint16 seq
uint8 msg_type
uint8 radar_data_type
uint32 session_handle
uint8 status
uint16 num_samples
uint16 block_size
uint8 bytes_per_tap
uint8[] raw_payload
```

Rationale:

- keeps the transport metadata available,
- preserves the full raw payload,
- avoids committing too early to one final DSP representation,
- supports future parser upgrades without changing the acquisition node.

## ROS2 Frame Publisher Node

Add a new node, separate from the configuration logic if needed, that:

- listens for UDP `RADAR_FRAME` packets,
- validates the envelope,
- extracts the raw SR250 payload,
- parses enough metadata to populate `num_samples`, `block_size`, and `bytes_per_tap`,
- publishes `UWB_Frame`.

This node is the runtime replacement for ad hoc scripts that only parse a dump file.

## ROS2 Parser and Archival Node

Add a second ROS2 node that subscribes to `UWB_Frame` and:

1. parses `raw_payload` using the observed SR250 block structure,
2. extracts per-block metadata,
3. decodes taps,
4. stores each frame to `.npz`.

### SR250 frame assumptions

Current observed format:

- common header once:
  - `session_handle[4]`
  - `status[1]`
  - `radar_data_type[1]`
  - `num_samples[2]`
  - `taps_per_sample[1]`
  - `rfu[1]`
- then `num_samples` repeated blocks:
  - `metadata[32]`
  - `sample_data[block_size - 32]`

### Tap decoding

Support both:

- `4 bytes per tap`: `real = int16`, `imag = int16`
- `3 bytes per tap`: `real = int16(byte0, byte1)`, `imag = sign-extended int8(byte2)`

### `.npz` contents

Each saved file should include at least:

- `raw_payload`
- `session_handle`
- `status`
- `radar_data_type`
- `num_samples`
- `block_size`
- `bytes_per_tap`
- `cir_counters`
- `rx_paths`
- `rx_antenna_ids`
- `tx_antenna_ids`
- `timestamps_rx`
- `timestamps_tx`
- `taps_real`
- `taps_imag`

Recommended shapes:

- `taps_real.shape = (num_samples, taps_per_block)`
- `taps_imag.shape = (num_samples, taps_per_block)`

## Endianness Rules

### Protocol layer

- Envelope fields are little-endian.
- TLV `tag` and `len` are little-endian.
- Integer values inside TLVs are little-endian.

### SR250 payload layer

- Reuse the same little-endian interpretation already used in STM32 UCI parsing.
- Signed 8-bit tap imag must be sign-extended only for the 3-byte tap mode.

These rules must be encoded explicitly in both Python and STM32 code paths.

## Testing Strategy

### Step 1: Host command encoding

- Unit-test TLV serialization in Python.
- Verify packet bytes against expected known vectors.

### Step 2: STM32 envelope and TLV parser

- Validate good packet, bad packet, partial packet, unsupported tag.
- Confirm correct ACK/ERROR behavior.

### Step 3: SR250 configuration path

- Send `SET_CONFIG_FULL`.
- Verify STM32 returns ACK.
- Verify SR250 config applies successfully.

### Step 4: Runtime updates

- Send `SET_PARAMS_PARTIAL` for a supported runtime field.
- Verify ACK and changed behavior.

### Step 5: Data return path

- Send `START_RADAR`.
- Verify STM32 emits `RADAR_FRAME`.
- Verify host node publishes `UWB_Frame`.

### Step 6: Parser and storage

- Verify parser node stores `.npz`.
- Load `.npz` in Python and verify array shapes and metadata consistency.

## Safety and Error Policy

- Never apply `SET_PARAMS_PARTIAL` before a valid full config has been accepted.
- Reject malformed envelopes before touching radar state.
- Preserve the current config if a partial update fails validation.
- Do not crash ROS2 nodes on bad frames; log warnings and drop invalid packets.
- Preserve raw frame bytes in ROS2 and `.npz` so parsing can be revisited later.

## Implementation Order

1. Update `uwb_node.py` to emit the new UDP protocol.
2. Add STM32 UDP envelope/TLV parser and ACK/ERROR path.
3. Bridge TLVs into `uci_radar_params_t` and runtime setters.
4. Wrap SR250 notifications into `RADAR_FRAME`.
5. Add `UWB_Frame.msg`.
6. Add ROS2 UDP-to-`UWB_Frame` publisher node.
7. Add ROS2 `UWB_Frame` parser and `.npz` writer node.

## Open Decisions Resolved

- Protocol style: extensible binary TLV.
- Command model: support both full config and partial update.
- ROS2 transport message: `UWB_Frame.msg` minimal raw-preserving schema.
- Storage target: `.npz` with both raw and decoded arrays.

## Residual Risk

The main technical risk is that future SR250 firmware or settings may change the observed per-block data layout. The design mitigates this by:

- preserving raw payloads end-to-end,
- keeping parsing in a dedicated ROS2 node,
- and separating raw frame transport from final decoded storage.

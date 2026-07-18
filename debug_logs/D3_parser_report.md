# D3 - Offline Parser Tests

Date: 2026-07-16

Implemented `new_uwb/new_uwb/protocol.py` (pure stdlib, no ROS2/rclpy import —
importable standalone) covering the `CMD_*` transport layer confirmed live in
D0: builders for `CMD_START_SESSION`/`CMD_SYSTEM_COMMAND`, parsers for
ACK/ERROR/SYSTEM_STATE/CIR-fragment, and a `CIRReassembler` that buffers
fragments until `last_fragment=1`.

Tests in `new_uwb/test/test_protocol.py`, run with:

```
cd new_uwb
python -m unittest test.test_protocol -v
```

Result: **25/25 passed.**

Coverage against the addendum's required list:

| Required | Test(s) |
|---|---|
| ACK parser `03 01` | `test_ack_for_start_session` |
| ERROR parser `04 <err_id>` | `test_known_error_id`, `test_unknown_error_id_falls_back_to_hex_name` |
| SYSTEM_STATE parser `06 <state>` | `test_system_ok`, `test_radar_session_done` |
| CIR fragment parser, valid `data_len` | `test_single_fragment_valid_data_len`, `test_non_last_fragment_flag` |
| CIR reassembly until `last_fragment=1` | `test_multi_fragment_frame_reassembles_in_order`, `test_max_size_fragment_matches_firmware_chunking` |
| Bad length rejection | `test_ack_bad_length_rejected`, `test_error_bad_length_rejected`, `test_too_short_rejected`, `test_data_len_mismatch_rejected`, `test_data_len_over_max_rejected` |
| Unknown command handling | `test_ack_wrong_cmd_id_rejected`, `test_wrong_cmd_id_rejected` (x2), `test_unknown_command` (`identify_packet`) |

`test_max_size_fragment_matches_firmware_chunking` specifically mirrors
`nucleo_udp_send()`'s `CMD_CIR_REPORT` loop in `udp_server.c:154-199`: a
payload longer than `MAX_DATA_PER_ETH_PACKET` (1184) splits into one
non-last max-size fragment plus a last remainder fragment, and the
reassembler must reconstruct the original bytes exactly. This is synthetic
data (no hardware capture available yet), but it encodes the exact same
splitting rule the firmware uses, so it's a meaningful check of the
reassembly logic itself, not just a tautology.

## What D3 deliberately does NOT cover

Reassembling a `CMD_CIR_REPORT` train yields the raw SR250 UCI
`RADAR_RX_NTF` payload — the *inner* CIR metadata/sample layout inside that
blob (session_handle/status/radar_data_type/num_samples/taps/... — the
`CIRMetadata` struct). That parsing is intentionally reused from
`sensors.sr250_protocol.parse_cir_udp_payload` rather than reimplemented (see
`NEW_UWB_PLAN.md` Phase 2). Confirmed by reading source that this reuse
should be valid: `uwb_sw/Core/Src/main.c:511-527`
(`app_uci_notification_handler`) sends the SR250 notification's raw
`payload`/`len` straight to `nucleo_udp_send(..., CMD_CIR_REPORT, payload,
len)`, and `boshUWBSTM32/Drivers/Src/main.c` (the known-working legacy_tlv
reference) does the exact same thing —
`uwb_udp_protocol_queue_radar_frame(payload, len)` with the same raw
`payload`/`len` from the same notification handler. Both firmwares forward
the identical raw NTF bytes; only the outer transport framing differs. This
is *not* independently verified against a real hardware capture yet — that
has to happen once a board is flashed and traffic is captured (D2/D6).

## Gate

**Passed**: parser reassembles synthetic CIR fragment trains correctly,
including the firmware's own max-fragment-size chunking rule.

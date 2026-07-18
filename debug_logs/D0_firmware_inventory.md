# D0 - Static Firmware Inventory

Date: 2026-07-16
Scope: `uwb_sw/Core` (verified against source, not assumed).

## Commands run

```powershell
rg -n "nucleo_udp_init|uwb_udp_protocol_init|CMD_CIR_REPORT|CMD_START_SESSION|nucleo_udp_send|destination_ip_addr" uwb_sw\Core
rg -n "MAX_DATA_PER_ETH_PACKET|NUCLEO_ETH_PORT|NUCLEO_REMOTE_PORT|CMD_" uwb_sw\Core\Inc\udp_server.h
```

Full output kept in the conversation; key hits below.

## Which protocol is actually wired into `main.c`

`CMD_*` (`udp_server.c` / `udp_server.h`). Evidence:

- `main.c:174` — `nucleo_udp_init();` called from `MX_LWIP_Init`/setup path.
- `main.c:524` — `nucleo_udp_send(NUCLEO_REMOTE_PORT, CMD_CIR_REPORT, payload, len);`
  sends CIR data.
- `main.c:301-434` — eight more `nucleo_udp_send(NUCLEO_REMOTE_PORT, err_report_cmd.cmd_id, ...)`
  call sites for `CMD_ERROR_REPORT` and `CMD_SYSTEM_STATE`.

`uwb_udp_protocol_init()` (the TLV-envelope path, `uwb_udp_protocol.c`) has **zero
call sites** anywhere in `uwb_sw/Core` outside its own definition
(`uwb_udp_protocol.c:299`) and header declaration
(`uwb_udp_protocol.h:46`). Confirmed dead code, not just "unused" — see the
compile-break note below.

## Which file owns UDP receive

`udp_server.c`: `nucleo_udp_init()` (line 50) binds a `udp_pcb` to the board's
static IP on `NUCLEO_ETH_PORT = 37249` and registers `udp_receive_callback`
(line 81) via `udp_recv()`. That callback handles inbound `CMD_START_SESSION`
(0x01) and `CMD_SYSTEM_COMMAND` (0x05, sub-command `CMD_SYSTEM_RESET = 0x01`),
ACKing both with `CMD_UDP_ETH_ACK` (0x03).

## Which file owns CIR transmit

`main.c` builds the CIR payload and calls `nucleo_udp_send(NUCLEO_REMOTE_PORT,
CMD_CIR_REPORT, payload, len)` (line 524). The actual UDP framing/fragmentation
(`udp_input_cmd_t`/`udp_cir_fragment_t` wrapping, `MAX_DATA_PER_ETH_PACKET = 1184`
byte fragments, `last_fragment` flag) happens inside `nucleo_udp_send()` in
`udp_server.c:130-202`.

## Hardcoded PC destination IP

`udp_server.c:57` — `destination_ip_addr.addr = 0x6601A8C0;`, used by
`udp_connect(send_upcb, &destination_ip_addr, 20000)` (line 62). Decoded as an IP
in the byte order this codebase uses elsewhere (`IP_ADDR4(a,b,c,d)` pattern):
**192.168.1.102**. This is the PC IP the board will actually send
`CMD_CIR_REPORT`/`CMD_UDP_ETH_ACK`/`CMD_ERROR_REPORT` datagrams to, regardless of
what IP `new_uwb` binds/listens on. **Action item before D2**: confirm the PC's
NIC connected to the board is actually configured as `192.168.1.102`, or the
firmware needs a source-level IP change and rebuild.

## Port map

| Direction | Port | Purpose |
|---|---|---|
| PC -> board | 37249 (`NUCLEO_ETH_PORT`) | `CMD_START_SESSION`, `CMD_SYSTEM_COMMAND` |
| board -> PC | 20000 (`NUCLEO_REMOTE_PORT`) | `CMD_CIR_REPORT`, `CMD_UDP_ETH_ACK`, `CMD_ERROR_REPORT`, `CMD_SYSTEM_STATE` (all multiplexed on the same single port — no separate ack port like legacy_tlv's `PC_LISTEN_ACK_PORT=20001`) |

## Known firmware defects found during inventory (not in original addendum draft)

1. **`uwb_udp_protocol.c` does not compile against the current `nucleo_udp_send`
   prototype.** Call sites at `uwb_udp_protocol.c:71`, `:86`, `:477` pass 3
   arguments (`reply_port, packet, packet_len`); `udp_server.h:40` declares 4
   required (`port, cmd_id, payload, payload_length`). Must be excluded from the
   CubeIDE build set, not conditionally.
2. **`CMD_SYSTEM_STATE` is built but never transmitted.** `main.c:301-303` and
   `main.c:424-426` call `nucleo_udp_send(NUCLEO_REMOTE_PORT, CMD_SYSTEM_STATE, ...)`,
   but the `switch (cmd_id)` in `nucleo_udp_send()` (`udp_server.c:134-200`) only
   has cases for `CMD_UDP_ETH_ACK`/`CMD_ERROR_REPORT` (grouped) and
   `CMD_CIR_REPORT`. No `case CMD_SYSTEM_STATE`, no `default`. Those two call
   sites currently return `send_status = 1` without allocating or sending a pbuf.

## Gate

Active protocol documented: **`CMD_*`**, matching the addendum's expected answer.
Proceeding to D1.

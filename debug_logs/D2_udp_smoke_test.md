# D2 - Minimal UDP Test Without ROS2

Date: 2026-07-16

## Status: script ready, live-hardware run BLOCKED (no board connected — see D1_flash_smoke_test.md)

Implemented `tools/uwb_cmd_debug.py`: dependency-free (stdlib only, imports
`new_uwb/new_uwb/protocol.py` directly by path — no ROS2 sourcing required).
Sends `CMD_START_SESSION`, listens on `0.0.0.0:20000`, decodes every
datagram (ACK/ERROR/SYSTEM_STATE/CIR fragment or UNKNOWN), prints a
human-readable line per datagram, and writes JSONL to
`debug_logs/D2_udp_smoke_test.jsonl`.

## Self-test performed (no hardware, proves the script itself works)

Ran with `--no-send --listen-seconds 1 --listen-port 29999` against an
otherwise-quiet port: started cleanly, produced the expected "0 datagrams"
summary and diagnostic checklist, exited 0. No crash, no import errors.
(Log written to a throwaway path, not committed — this was a script
self-check, not a protocol capture.)

## Real gate — not yet run

The actual D2 gate ("start command bytes visible in the log, at least one
ACK/ERROR/CIR fragment decoded") requires the physical board, which is not
connected to this machine right now (see `D1_flash_smoke_test.md`).

## One environment note for whoever runs this against real hardware

This machine is also running the `uwb_nxp` Docker container with
`0.0.0.0:20000-20001->20000-20001/udp` published (see `compose.*.yaml`). Run
`tools/uwb_cmd_debug.py` **inside** that container (`make uwb.shell`, then
`python3 tools/uwb_cmd_debug.py ...`), not directly on the Windows host —
otherwise the host and the container both have something on port 20000 and
it's not obvious which one actually receives real board traffic arriving on
the physical NIC. A host-side bind test during this session succeeded
(`BIND_OK` on 0.0.0.0:20000) even with the container's port mapping active,
which confirms there's no OS-level exclusive-lock protecting against this —
i.e. it's easy to accidentally run the debug script somewhere that silently
doesn't see the board's packets.

## Gate

**Not evaluated — blocked on hardware**, same reason as D1's flash gate.
Script itself is verified working (self-test passed); nothing left to do
here except run it once a board is flashed and connected.

#!/usr/bin/env python3
"""Minimal, dependency-free UDP smoke test for the uwb_sw (NXP board) protocol.

No ROS2 needed. Sends CMD_START_SESSION to the board, then listens on
0.0.0.0:20000 and prints/logs every datagram it receives, decoded with the
same new_uwb.protocol module the ROS2 nodes use.

Usage:
    python tools/uwb_cmd_debug.py --board-ip 192.168.1.10 [--setting-idx 0]
                                   [--duration-ms 10000] [--listen-seconds 30]

Writes JSONL to debug_logs/D2_udp_smoke_test.jsonl (one JSON object per line,
per the addendum's D2 spec) in addition to printing to stdout.
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "new_uwb"))

from new_uwb import protocol as p  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_LOG_PATH = REPO_ROOT / "debug_logs" / "D2_udp_smoke_test.jsonl"


def decode_datagram(data: bytes) -> dict:
    """Best-effort decode; never raises — malformed packets are logged as such."""
    name = p.identify_packet(data)
    result = {"cmd_name": name, "raw_hex": data.hex(" ")}

    try:
        if name == "ACK":
            result["decoded"] = p.parse_ack_packet(data)
        elif name == "ERROR_REPORT":
            result["decoded"] = p.parse_error_packet(data)
        elif name == "SYSTEM_STATE":
            result["decoded"] = p.parse_system_state_packet(data)
        elif name == "CIR_REPORT":
            frag = p.parse_cir_fragment_packet(data)
            result["decoded"] = {
                "last_fragment": frag["last_fragment"],
                "data_len": frag["data_len"],
            }
        else:
            result["decoded"] = None
            result["malformed"] = name.startswith("UNKNOWN")
    except ValueError as exc:
        result["decoded"] = None
        result["malformed"] = True
        result["parse_error"] = str(exc)

    return result


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--board-ip", default="192.168.1.10", help="uwb_sw board IP (command port target)")
    ap.add_argument("--board-port", type=int, default=p.NUCLEO_ETH_PORT)
    ap.add_argument("--listen-ip", default="0.0.0.0")
    ap.add_argument("--listen-port", type=int, default=p.NUCLEO_REMOTE_PORT)
    ap.add_argument("--setting-idx", type=int, default=0)
    ap.add_argument("--duration-ms", type=int, default=10_000)
    ap.add_argument("--listen-seconds", type=float, default=30.0, help="how long to listen after sending START_SESSION")
    ap.add_argument("--no-send", action="store_true", help="only listen, do not send CMD_START_SESSION")
    ap.add_argument("--log-path", default=str(DEFAULT_LOG_PATH))
    args = ap.parse_args()

    log_path = Path(args.log_path)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.listen_ip, args.listen_port))
    sock.settimeout(0.5)

    print(f"Listening on {args.listen_ip}:{args.listen_port}, logging to {log_path}")

    with log_path.open("a", encoding="utf-8") as log_file:
        def log(record: dict) -> None:
            record["ts"] = time.time()
            log_file.write(json.dumps(record) + "\n")
            log_file.flush()

        log({"event": "start", "args": vars(args)})

        if not args.no_send:
            packet = p.build_start_session_packet(args.setting_idx, args.duration_ms)
            sock.sendto(packet, (args.board_ip, args.board_port))
            print(f"Sent CMD_START_SESSION to {args.board_ip}:{args.board_port} "
                  f"setting_idx={args.setting_idx} duration_ms={args.duration_ms} "
                  f"raw={packet.hex(' ')}")
            log({
                "event": "sent",
                "cmd_name": "START_SESSION",
                "dest": f"{args.board_ip}:{args.board_port}",
                "setting_idx": args.setting_idx,
                "duration_ms": args.duration_ms,
                "raw_hex": packet.hex(" "),
            })

        deadline = time.monotonic() + args.listen_seconds
        counts: dict = {}
        datagram_count = 0

        while time.monotonic() < deadline:
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue

            datagram_count += 1
            decoded = decode_datagram(data)
            counts[decoded["cmd_name"]] = counts.get(decoded["cmd_name"], 0) + 1

            line = (
                f"[{time.strftime('%H:%M:%S')}] {addr[0]}:{addr[1]} "
                f"len={len(data)} cmd={decoded['cmd_name']} "
                f"decoded={decoded.get('decoded')} raw={decoded['raw_hex']}"
            )
            print(line)

            log({
                "event": "recv",
                "src": f"{addr[0]}:{addr[1]}",
                "len": len(data),
                **decoded,
            })

        summary = {"event": "summary", "datagram_count": datagram_count, "counts": counts}
        log(summary)
        print(f"\nDone. {datagram_count} datagram(s) received. Counts: {counts}")
        if datagram_count == 0:
            print(
                "No datagrams received at all. Check (in order): board flashed and "
                "powered, Ethernet link up, PC NIC on the board's subnet, PC IP "
                "actually set to 192.168.1.102 (hardcoded in "
                "udp_server.c:57 as of this writing — see "
                "debug_logs/D0_firmware_inventory.md), firewall allowing inbound "
                f"UDP {args.listen_port}, and that nothing else on this machine "
                f"already owns port {args.listen_port} (e.g. the new_uwb ROS2 "
                "publisher node or the sensors/legacy_tlv stack)."
            )

    sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

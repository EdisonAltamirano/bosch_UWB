from __future__ import annotations

import argparse
import json
import socket

try:
    from sensors.sr250_protocol import parse_cir_udp_payload, load_wireshark_hex_dump
except ImportError:
    from sr250_protocol import parse_cir_udp_payload, load_wireshark_hex_dump


def print_summary(parsed):
    print("=== Datagram summary ===")
    print(
        json.dumps(
            {
                "session_handle": parsed["session_handle"],
                "status": parsed["status"],
                "radar_data_type": parsed["radar_data_type"],
                "num_samples": parsed["num_samples"],
                "taps_per_sample": parsed["taps_per_sample"],
                "block_size": parsed["block_size"],
                "bytes_per_tap": parsed["bytes_per_tap"],
                "actual_cir_taps_per_block": parsed["actual_cir_taps_per_block"],
            },
            indent=2,
        )
    )

    for idx, sample in enumerate(parsed["samples"]):
        meta = sample["metadata"]
        print(f"\n=== Sample block {idx} ===")
        print(
            json.dumps(
                {
                    "cir_counter": meta["cir_counter"],
                    "rx_timestamp": meta["rx_timestamp"],
                    "tx_timestamp": meta["tx_timestamp"],
                    "rx_path": meta["rx_path"],
                    "rx_antenna_id": meta["rx_antenna_id"],
                    "tx_antenna_id": meta["tx_antenna_id"],
                    "num_cir_taps": sample["num_cir_taps"],
                    "cir_start_offset": meta["cir_start_offset"],
                },
                indent=2,
            )
        )
        print("primeros 10 taps:")
        for tap_idx, (re_part, im_part) in enumerate(sample["taps"][:10]):
            print(f"  tap[{tap_idx}] = real={re_part}, imag={im_part}")


def recv_and_parse_udp(listen_ip: str = "0.0.0.0", listen_port: int = 20000) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((listen_ip, listen_port))
    print(f"Escuchando UDP en {listen_ip}:{listen_port}")

    while True:
        data, addr = sock.recvfrom(65535)
        print(f"\nRecibido {len(data)} bytes desde {addr}")

        try:
            parsed = parse_cir_udp_payload(data)
            print_summary(parsed)
        except Exception as exc:
            print(f"Error parseando paquete: {exc}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dump", help="Ruta al dump hexadecimal de Wireshark")
    parser.add_argument("--listen-ip", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=20000)
    args = parser.parse_args()

    if args.dump:
        payload = load_wireshark_hex_dump(args.dump)
        parsed = parse_cir_udp_payload(payload)
        print_summary(parsed)
    else:
        recv_and_parse_udp(listen_ip=args.listen_ip, listen_port=args.port)


if __name__ == "__main__":
    main()

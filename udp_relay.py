#!/usr/bin/env python3
"""
UDP relay — run this on the Mac host (outside Docker).

Bridges the Docker container's ROS2 nodes to the physical STM32 board
connected via Ethernet (en6), working around Docker Desktop Mac's inability
to route container UDP traffic to physical network interfaces.

Flow:
  Container ──▶ host.docker.internal:37249 ──▶ [this relay] ──▶ STM32:37249
  STM32 ──ACK/frames──▶ Mac:20001/20000 ──▶ Docker port map ──▶ Container

Usage:
  python3 udp_relay.py

Then set STM32_IP = "host.docker.internal" in sensors/launch/sensors.launch.py.
"""

import socket
import threading

STM32_IP   = "192.168.1.10"
STM32_PORT = 37249
LISTEN_PORT = 37249   # same port the container sends to on host.docker.internal


def relay():
    # Socket that listens for commands from the container
    listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listen_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listen_sock.bind(("0.0.0.0", LISTEN_PORT))
    print(f"[relay] Listening on 0.0.0.0:{LISTEN_PORT} → forwarding to {STM32_IP}:{STM32_PORT}")

    # Socket that talks to the STM32
    stm32_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stm32_sock.settimeout(0.1)

    container_addr = None

    def recv_from_stm32():
        """Forward STM32 replies back to the container."""
        while True:
            try:
                data, _ = stm32_sock.recvfrom(65535)
                if container_addr:
                    listen_sock.sendto(data, container_addr)
                    print(f"[relay] STM32 → container ({len(data)} bytes)")
            except socket.timeout:
                continue
            except Exception as exc:
                print(f"[relay] stm32_sock error: {exc}")

    threading.Thread(target=recv_from_stm32, daemon=True).start()

    while True:
        try:
            data, addr = listen_sock.recvfrom(65535)
            container_addr = addr
            stm32_sock.sendto(data, (STM32_IP, STM32_PORT))
            print(f"[relay] container → STM32 ({len(data)} bytes) from {addr}")
        except Exception as exc:
            print(f"[relay] listen_sock error: {exc}")


if __name__ == "__main__":
    relay()

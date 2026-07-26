#!/usr/bin/env python3
"""Publish the versioned 3D high/low command boundary used by a2_deploy."""

from __future__ import annotations

import argparse
import math
import socket
import time
from typing import Sequence


def encode_navigation_action(vx: float, vy: float, yaw_rate: float) -> bytes:
    values = (float(vx), float(vy), float(yaw_rate))
    if not all(math.isfinite(value) for value in values):
        raise ValueError("navigation actions must be finite")
    return f"A2NAV1 {values[0]:.9g} {values[1]:.9g} {values[2]:.9g}".encode(
        "ascii"
    )


class A2NavigationPublisher:
    """Small client used by a high-policy process to submit 3D actions."""

    def __init__(self, host: str = "127.0.0.1", port: int = 15000) -> None:
        if not 1024 <= port <= 65535:
            raise ValueError("port must be in [1024, 65535]")
        self._destination = (host, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def publish(self, action: Sequence[float]) -> None:
        if len(action) != 3:
            raise ValueError("navigation action must contain vx, vy, yaw_rate")
        payload = encode_navigation_action(action[0], action[1], action[2])
        self._socket.sendto(payload, self._destination)

    def close(self, send_zero: bool = True) -> None:
        if self._socket.fileno() < 0:
            return
        if send_zero:
            zero = encode_navigation_action(0.0, 0.0, 0.0)
            for _ in range(3):
                self._socket.sendto(zero, self._destination)
        self._socket.close()

    def __enter__(self) -> "A2NavigationPublisher":
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Send [vx, vy, yaw_rate] actions to a2_deploy --command-source high"
        )
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=15000)
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to publish; 0 runs until Ctrl+C",
    )
    args = parser.parse_args()

    if not 1024 <= args.port <= 65535:
        parser.error("--port must be in [1024, 65535]")
    if not math.isfinite(args.rate) or args.rate <= 0.0:
        parser.error("--rate must be positive and finite")
    if not math.isfinite(args.duration) or args.duration < 0.0:
        parser.error("--duration must be non-negative and finite")

    payload = encode_navigation_action(args.vx, args.vy, args.yaw)
    period = 1.0 / args.rate
    started = time.monotonic()
    next_send = started
    print(
        f"Publishing {payload.decode()} to udp://{args.host}:{args.port} "
        f"at {args.rate:g} Hz"
    )
    action = (args.vx, args.vy, args.yaw)
    with A2NavigationPublisher(args.host, args.port) as publisher:
        try:
            while args.duration == 0.0 or time.monotonic() - started < args.duration:
                publisher.publish(action)
                next_send += period
                time.sleep(max(0.0, next_send - time.monotonic()))
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

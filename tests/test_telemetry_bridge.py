#!/usr/bin/env python3
"""Regression tests for the isolated A2 telemetry wire contract."""

from __future__ import annotations

import json
import socket
import sys
import threading
import unittest
from dataclasses import replace
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "sim"))

from a2_telemetry_bridge import (  # noqa: E402
    A2TelemetryError,
    A2TelemetryGate,
    A2TelemetryPublisher,
    A2TelemetryReason,
    A2TelemetryReceiver,
    A2TelemetrySample,
)


def sample(
    *,
    session_id: str = "session-a",
    sequence: int = 7,
    state_sequence: int = 101,
    state_time: int = 1_000_000_000,
    velocity_time: int = 1_001_000_000,
    commit_time: int = 1_002_000_000,
    publish_time: int = 1_003_000_000,
    motion_permitted: bool = True,
) -> A2TelemetrySample:
    return A2TelemetrySample(
        session_id=session_id,
        sequence=sequence,
        state_sequence=state_sequence,
        state_capture_time_ns=state_time,
        base_velocity_capture_time_ns=velocity_time,
        policy_commit_time_ns=commit_time,
        publish_time_ns=publish_time,
        filtered_command=(0.4, -0.2, 0.5),
        projected_gravity_b=(0.0, 0.0, -1.0),
        base_linear_velocity_b=(0.3, -0.1, 0.0),
        base_angular_velocity_b=(0.0, 0.0, 0.2),
        low_level_policy_action=tuple(float(index) / 10.0 for index in range(12)),
        base_velocity_source="unitree_sport_state_body",
        motion_permitted=motion_permitted,
        phase="CTRL" if motion_permitted else "FAULT",
        fault="" if motion_permitted else "low_state_timeout",
    )


class CodecTests(unittest.TestCase):
    def test_round_trip_is_exact_and_owned(self) -> None:
        original = sample()
        payload = bytearray(original.encode())
        decoded = A2TelemetrySample.decode(payload)
        payload[:] = b"{}"
        self.assertEqual(decoded, original)
        self.assertLess(len(original.encode()), 4096)

    def test_missing_extra_duplicate_and_nonfinite_json_are_rejected(self) -> None:
        mapping = sample().as_dict()
        mapping.pop("filtered_command")
        with self.assertRaisesRegex(A2TelemetryError, "missing"):
            A2TelemetrySample.decode(json.dumps(mapping).encode("ascii"))

        mapping = sample().as_dict()
        mapping["extra"] = 1
        with self.assertRaisesRegex(A2TelemetryError, "extra"):
            A2TelemetrySample.decode(json.dumps(mapping).encode("ascii"))

        payload = sample().encode().decode("ascii")
        duplicate = payload[:-1] + ',"sequence":8}'
        with self.assertRaisesRegex(A2TelemetryError, "duplicate"):
            A2TelemetrySample.decode(duplicate.encode("ascii"))

        payload = sample().encode().decode("ascii").replace(
            '"filtered_command":[0.4,-0.2,0.5]',
            '"filtered_command":[NaN,-0.2,0.5]',
        )
        with self.assertRaisesRegex(A2TelemetryError, "non-standard"):
            A2TelemetrySample.decode(payload.encode("ascii"))

    def test_required_measured_fields_fail_closed(self) -> None:
        with self.assertRaisesRegex(A2TelemetryError, "physical command envelope"):
            replace(sample(), filtered_command=(1.01, 0.0, 0.0))
        with self.assertRaisesRegex(A2TelemetryError, "measured state estimator"):
            replace(sample(), base_velocity_source="commanded_velocity")
        with self.assertRaisesRegex(A2TelemetryError, "norm"):
            replace(sample(), projected_gravity_b=(0.0, 0.0, 0.0))
        with self.assertRaisesRegex(A2TelemetryError, "must not follow"):
            replace(sample(), state_capture_time_ns=1_004_000_000)
        with self.assertRaisesRegex(A2TelemetryError, "policy SHA"):
            replace(sample(), low_level_policy_sha256="0" * 64)


class GateTests(unittest.TestCase):
    def test_contiguous_fresh_stream_is_accepted(self) -> None:
        gate = A2TelemetryGate()
        first = gate.evaluate(sample(), 1_004_000_000)
        second_sample = sample(
            sequence=8,
            state_sequence=102,
            state_time=1_020_000_000,
            velocity_time=1_021_000_000,
            commit_time=1_022_000_000,
            publish_time=1_023_000_000,
        )
        second = gate.evaluate(second_sample, 1_024_000_000)
        self.assertTrue(first.accepted)
        self.assertTrue(second.accepted)
        self.assertEqual(second.sample, second_sample)

    def test_stale_gap_and_motion_block_latch_restart(self) -> None:
        stale_gate = A2TelemetryGate()
        stale = stale_gate.evaluate(sample(), 1_100_000_000)
        self.assertEqual(stale.reason, A2TelemetryReason.TIMING)
        self.assertTrue(stale.must_stop)

        gap_gate = A2TelemetryGate()
        self.assertTrue(gap_gate.evaluate(sample(), 1_004_000_000).accepted)
        gap = sample(
            sequence=9,
            state_sequence=103,
            state_time=1_020_000_000,
            velocity_time=1_021_000_000,
            commit_time=1_022_000_000,
            publish_time=1_023_000_000,
        )
        self.assertEqual(
            gap_gate.evaluate(gap, 1_024_000_000).reason,
            A2TelemetryReason.ORDER,
        )
        self.assertEqual(
            gap_gate.evaluate(gap, 1_024_000_001).reason,
            A2TelemetryReason.RESET_REQUIRED,
        )

        blocked_gate = A2TelemetryGate()
        blocked = blocked_gate.evaluate(
            sample(motion_permitted=False), 1_004_000_000
        )
        self.assertEqual(blocked.reason, A2TelemetryReason.MOTION_BLOCKED)

    def test_rearm_requires_unused_new_session(self) -> None:
        gate = A2TelemetryGate()
        self.assertTrue(gate.evaluate(sample(), 1_004_000_000).accepted)
        invalid = gate.invalidate("producer fault")
        self.assertTrue(invalid.must_stop)

        same = sample(
            sequence=8,
            state_sequence=102,
            state_time=1_020_000_000,
            velocity_time=1_021_000_000,
            commit_time=1_022_000_000,
            publish_time=1_023_000_000,
        )
        self.assertEqual(
            gate.evaluate(same, 1_024_000_000, rearm=True).reason,
            A2TelemetryReason.SESSION,
        )
        fresh = replace(same, session_id="session-b", sequence=0)
        decision = gate.evaluate(fresh, 1_024_000_001, rearm=True)
        self.assertTrue(decision.accepted)


class UdpTests(unittest.TestCase):
    def test_empty_poll_is_an_explicit_stop(self) -> None:
        receiver = A2TelemetryReceiver(port=0)
        try:
            decision = receiver.receive_latest(1_004_000_000)
            self.assertIsNotNone(decision)
            assert decision is not None
            self.assertTrue(decision.must_stop)
            self.assertEqual(decision.reason, A2TelemetryReason.TIMING)
        finally:
            receiver.close()

    def test_loopback_publish_receive(self) -> None:
        receiver = A2TelemetryReceiver(port=0)
        publisher = A2TelemetryPublisher(port=receiver.port)
        try:
            publisher.publish(sample())
            decision = receiver.receive_latest(1_004_000_000)
            self.assertIsNotNone(decision)
            assert decision is not None
            self.assertTrue(decision.accepted)
            self.assertEqual(decision.sample, sample())
        finally:
            publisher.close()
            receiver.close()

    def test_receiver_rejects_malformed_and_backlog(self) -> None:
        receiver = A2TelemetryReceiver(port=0, max_datagrams_per_poll=2)
        raw = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            raw.sendto(b"{}", ("127.0.0.1", receiver.port))
            decision = receiver.receive_latest(1_004_000_000)
            self.assertIsNotNone(decision)
            assert decision is not None
            self.assertEqual(decision.reason, A2TelemetryReason.MALFORMED)
        finally:
            raw.close()
            receiver.close()

        receiver = A2TelemetryReceiver(port=0, max_datagrams_per_poll=2)
        raw = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            for _ in range(3):
                raw.sendto(sample().encode(), ("127.0.0.1", receiver.port))
            decision = receiver.receive_latest(1_004_000_000)
            self.assertIsNotNone(decision)
            assert decision is not None
            self.assertTrue(decision.must_stop)
            self.assertEqual(decision.reason, A2TelemetryReason.BACKLOG)
        finally:
            raw.close()
            receiver.close()

    def test_publisher_close_is_serialized_with_inflight_send(self) -> None:
        entered = threading.Event()
        release = threading.Event()

        class BlockingSocket:
            def __init__(self, *_: object) -> None:
                self.events: list[str] = []

            def sendto(self, payload: bytes, destination: tuple[str, int]) -> int:
                self.events.append("send_enter")
                entered.set()
                release.wait(timeout=2.0)
                self.events.append("send_exit")
                return len(payload)

            def close(self) -> None:
                self.events.append("close")

        fake = BlockingSocket()
        publisher = A2TelemetryPublisher(
            socket_factory=lambda *_: fake  # type: ignore[arg-type]
        )
        publish_thread = threading.Thread(target=publisher.publish, args=(sample(),))
        close_thread = threading.Thread(target=publisher.close)
        publish_thread.start()
        self.assertTrue(entered.wait(timeout=1.0))
        close_thread.start()
        self.assertNotIn("close", fake.events)
        release.set()
        publish_thread.join(timeout=1.0)
        close_thread.join(timeout=1.0)
        self.assertEqual(fake.events, ["send_enter", "send_exit", "close"])
        with self.assertRaisesRegex(RuntimeError, "closed"):
            publisher.publish(sample())

    def test_non_loopback_endpoints_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "loopback"):
            A2TelemetryPublisher(host="192.0.2.1")
        with self.assertRaisesRegex(ValueError, "loopback"):
            A2TelemetryReceiver(host="192.0.2.1")


if __name__ == "__main__":
    unittest.main()

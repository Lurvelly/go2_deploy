#!/usr/bin/env python3
"""Strict loopback JSON/UDP contract for A2 policy telemetry.

The codec deliberately has no defaults for robot-state fields.  A producer
must obtain the filtered command and raw policy action from the low-level
controller, and base velocity from a timestamped estimator.  Missing or stale
evidence is rejected instead of being replaced with commands, joint state, or
zeros.
"""

from __future__ import annotations

import ipaddress
import json
import math
import re
import socket
import threading
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Mapping, Sequence


A2_TELEMETRY_SCHEMA = "A2TEL1"
A2_TELEMETRY_CLOCK = "host_monotonic"
A2_LOW_LEVEL_CONTRACT_ID = "a2_45d_project_v0"
A2_LOW_LEVEL_POLICY_SHA256 = (
    "886a653beb628ec09b3287b0e0db79279756e535037679f06d05c29d243b6cc8"
)
A2_TELEMETRY_MAX_DATAGRAM_BYTES = 4096
A2_TELEMETRY_DEFAULT_PORT = 15001

_SESSION_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}\Z")
_SOURCE_PATTERN = re.compile(r"[a-z][a-z0-9_]{0,63}\Z")
_FORBIDDEN_VELOCITY_SOURCES = frozenset(
    {
        "command",
        "commanded_velocity",
        "constant_zero",
        "joint_position_proxy",
        "target_position_proxy",
        "unknown",
        "zeros",
    }
)


class A2TelemetryError(ValueError):
    """Raised when an A2 telemetry packet violates the wire contract."""


class A2TelemetryReason(str, Enum):
    ACCEPTED = "accepted"
    MALFORMED = "malformed"
    IDENTITY = "identity"
    TIMING = "timing"
    ORDER = "order"
    SESSION = "session"
    MOTION_BLOCKED = "motion_blocked"
    RESET_REQUIRED = "reset_required"
    BACKLOG = "backlog"
    CLOSED = "closed"


def _integer(value: object, name: str, *, positive: bool = False) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise A2TelemetryError(f"{name} must be an integer")
    if value < (1 if positive else 0):
        qualifier = "positive" if positive else "non-negative"
        raise A2TelemetryError(f"{name} must be {qualifier}")
    if value > (1 << 63) - 1:
        raise A2TelemetryError(f"{name} exceeds signed 64-bit range")
    return value


def _token(value: object, name: str, pattern: re.Pattern[str]) -> str:
    if not isinstance(value, str) or pattern.fullmatch(value) is None:
        raise A2TelemetryError(f"{name} has invalid syntax")
    return value


def _vector(value: object, length: int, name: str) -> tuple[float, ...]:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(
        value, Sequence
    ):
        raise A2TelemetryError(f"{name} must be an array")
    if len(value) != length:
        raise A2TelemetryError(f"{name} must contain exactly {length} values")
    result: list[float] = []
    for index, item in enumerate(value):
        if isinstance(item, bool) or not isinstance(item, (int, float)):
            raise A2TelemetryError(f"{name}[{index}] must be numeric")
        number = float(item)
        if not math.isfinite(number):
            raise A2TelemetryError(f"{name}[{index}] must be finite")
        result.append(number)
    return tuple(result)


def _reject_json_constant(value: str) -> None:
    raise A2TelemetryError(f"non-standard JSON number is forbidden: {value}")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise A2TelemetryError(f"duplicate JSON key: {key}")
        result[key] = value
    return result


@dataclass(frozen=True)
class A2TelemetrySample:
    """One controller-owned 50 Hz sample in the host monotonic clock."""

    session_id: str
    sequence: int
    state_sequence: int
    state_capture_time_ns: int
    base_velocity_capture_time_ns: int
    policy_commit_time_ns: int
    publish_time_ns: int
    filtered_command: tuple[float, float, float]
    projected_gravity_b: tuple[float, float, float]
    base_linear_velocity_b: tuple[float, float, float]
    base_angular_velocity_b: tuple[float, float, float]
    low_level_policy_action: tuple[
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
        float,
    ]
    base_velocity_source: str
    motion_permitted: bool
    phase: str
    fault: str
    schema: str = A2_TELEMETRY_SCHEMA
    clock_domain: str = A2_TELEMETRY_CLOCK
    low_level_contract_id: str = A2_LOW_LEVEL_CONTRACT_ID
    low_level_policy_sha256: str = A2_LOW_LEVEL_POLICY_SHA256

    def __post_init__(self) -> None:
        if self.schema != A2_TELEMETRY_SCHEMA:
            raise A2TelemetryError("schema must be A2TEL1")
        if self.clock_domain != A2_TELEMETRY_CLOCK:
            raise A2TelemetryError("clock_domain must be host_monotonic")
        _token(self.session_id, "session_id", _SESSION_PATTERN)
        _integer(self.sequence, "sequence")
        _integer(self.state_sequence, "state_sequence")
        state_time = _integer(
            self.state_capture_time_ns, "state_capture_time_ns", positive=True
        )
        velocity_time = _integer(
            self.base_velocity_capture_time_ns,
            "base_velocity_capture_time_ns",
            positive=True,
        )
        commit_time = _integer(
            self.policy_commit_time_ns, "policy_commit_time_ns", positive=True
        )
        publish_time = _integer(
            self.publish_time_ns, "publish_time_ns", positive=True
        )
        if state_time > commit_time or velocity_time > commit_time:
            raise A2TelemetryError(
                "state and base-velocity captures must not follow policy commit"
            )
        if commit_time > publish_time:
            raise A2TelemetryError("policy commit must not follow publish time")

        vectors = (
            ("filtered_command", 3),
            ("projected_gravity_b", 3),
            ("base_linear_velocity_b", 3),
            ("base_angular_velocity_b", 3),
            ("low_level_policy_action", 12),
        )
        for name, length in vectors:
            object.__setattr__(self, name, _vector(getattr(self, name), length, name))

        lower = (-0.5, -0.5, -1.0)
        upper = (1.0, 0.5, 1.0)
        if any(
            value < lower[index] or value > upper[index]
            for index, value in enumerate(self.filtered_command)
        ):
            raise A2TelemetryError(
                "filtered_command exceeds the A2 physical command envelope"
            )
        gravity_norm = math.sqrt(sum(value * value for value in self.projected_gravity_b))
        if not 0.8 <= gravity_norm <= 1.2:
            raise A2TelemetryError("projected_gravity_b norm is outside [0.8, 1.2]")
        if any(abs(value) > 10.0 for value in self.base_linear_velocity_b):
            raise A2TelemetryError("base_linear_velocity_b exceeds 10 m/s")
        if any(abs(value) > 20.0 for value in self.base_angular_velocity_b):
            raise A2TelemetryError("base_angular_velocity_b exceeds 20 rad/s")
        if any(abs(value) > 100.0 for value in self.low_level_policy_action):
            raise A2TelemetryError("low_level_policy_action exceeds the sanity bound")

        source = _token(
            self.base_velocity_source,
            "base_velocity_source",
            _SOURCE_PATTERN,
        )
        if source in _FORBIDDEN_VELOCITY_SOURCES:
            raise A2TelemetryError(
                "base_velocity_source must identify a measured state estimator"
            )
        if not isinstance(self.motion_permitted, bool):
            raise A2TelemetryError("motion_permitted must be a boolean")
        _token(self.phase, "phase", _SESSION_PATTERN)
        if not isinstance(self.fault, str) or len(self.fault) > 128:
            raise A2TelemetryError("fault must be a string no longer than 128 bytes")
        if self.low_level_contract_id != A2_LOW_LEVEL_CONTRACT_ID:
            raise A2TelemetryError("low-level contract identity mismatch")
        if self.low_level_policy_sha256 != A2_LOW_LEVEL_POLICY_SHA256:
            raise A2TelemetryError("low-level policy SHA-256 mismatch")

    def as_dict(self) -> dict[str, object]:
        return {
            "base_angular_velocity_b": list(self.base_angular_velocity_b),
            "base_linear_velocity_b": list(self.base_linear_velocity_b),
            "base_velocity_capture_time_ns": self.base_velocity_capture_time_ns,
            "base_velocity_source": self.base_velocity_source,
            "clock_domain": self.clock_domain,
            "fault": self.fault,
            "filtered_command": list(self.filtered_command),
            "low_level_contract_id": self.low_level_contract_id,
            "low_level_policy_action": list(self.low_level_policy_action),
            "low_level_policy_sha256": self.low_level_policy_sha256,
            "motion_permitted": self.motion_permitted,
            "phase": self.phase,
            "policy_commit_time_ns": self.policy_commit_time_ns,
            "projected_gravity_b": list(self.projected_gravity_b),
            "publish_time_ns": self.publish_time_ns,
            "schema": self.schema,
            "sequence": self.sequence,
            "session_id": self.session_id,
            "state_capture_time_ns": self.state_capture_time_ns,
            "state_sequence": self.state_sequence,
        }

    def encode(self) -> bytes:
        payload = json.dumps(
            self.as_dict(),
            allow_nan=False,
            ensure_ascii=True,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("ascii")
        if len(payload) > A2_TELEMETRY_MAX_DATAGRAM_BYTES:
            raise A2TelemetryError("telemetry datagram exceeds the size limit")
        return payload

    @classmethod
    def decode(cls, payload: bytes | bytearray | memoryview) -> "A2TelemetrySample":
        raw = bytes(payload)
        if not raw or len(raw) > A2_TELEMETRY_MAX_DATAGRAM_BYTES:
            raise A2TelemetryError("telemetry datagram has an invalid size")
        try:
            text = raw.decode("ascii")
        except UnicodeDecodeError as error:
            raise A2TelemetryError("telemetry datagram must be ASCII JSON") from error
        try:
            decoded = json.loads(
                text,
                object_pairs_hook=_unique_object,
                parse_constant=_reject_json_constant,
            )
        except (json.JSONDecodeError, TypeError) as error:
            raise A2TelemetryError("telemetry datagram is not valid JSON") from error
        if not isinstance(decoded, Mapping):
            raise A2TelemetryError("telemetry JSON root must be an object")
        expected = set(cls.__dataclass_fields__)
        actual = set(decoded)
        if actual != expected:
            missing = sorted(expected - actual)
            extra = sorted(actual - expected)
            raise A2TelemetryError(
                f"telemetry JSON keys differ; missing={missing}, extra={extra}"
            )
        return cls(**decoded)


@dataclass(frozen=True)
class A2TelemetryGateConfig:
    max_transport_age_ns: int = 40_000_000
    max_state_age_ns: int = 40_000_000
    max_velocity_age_ns: int = 40_000_000
    max_state_velocity_skew_ns: int = 40_000_000
    max_future_skew_ns: int = 1_000_000

    def __post_init__(self) -> None:
        for name in self.__dataclass_fields__:
            _integer(getattr(self, name), name)


@dataclass(frozen=True)
class A2TelemetryDecision:
    accepted: bool
    must_stop: bool
    reason: A2TelemetryReason
    detail: str
    sample: A2TelemetrySample | None

    def __post_init__(self) -> None:
        if self.accepted == self.must_stop:
            raise ValueError("accepted and must_stop must be logical opposites")
        if self.accepted != (self.sample is not None):
            raise ValueError("only accepted decisions may carry a sample")


class A2TelemetryGate:
    """Stateful order, freshness, session, and motion-authority gate."""

    def __init__(self, config: A2TelemetryGateConfig | None = None) -> None:
        self.config = config or A2TelemetryGateConfig()
        if not isinstance(self.config, A2TelemetryGateConfig):
            raise TypeError("config must be an A2TelemetryGateConfig")
        self._lock = threading.RLock()
        self._session_id: str | None = None
        self._used_sessions: set[str] = set()
        self._last_sequence: int | None = None
        self._last_state_sequence: int | None = None
        self._last_state_capture_time_ns: int | None = None
        self._last_velocity_capture_time_ns: int | None = None
        self._last_publish_time_ns: int | None = None
        self._last_receive_time_ns: int | None = None
        self._restart_required = False

    def _failed(
        self,
        reason: A2TelemetryReason,
        detail: str,
        *,
        latch: bool = True,
    ) -> A2TelemetryDecision:
        if latch:
            self._restart_required = True
        return A2TelemetryDecision(False, True, reason, detail, None)

    def invalidate(
        self,
        detail: str,
        *,
        reason: A2TelemetryReason = A2TelemetryReason.MALFORMED,
    ) -> A2TelemetryDecision:
        with self._lock:
            if not isinstance(reason, A2TelemetryReason):
                raise TypeError("reason must be an A2TelemetryReason")
            return self._failed(reason, detail)

    def evaluate(
        self,
        sample: A2TelemetrySample,
        receive_time_ns: int,
        *,
        rearm: bool = False,
    ) -> A2TelemetryDecision:
        with self._lock:
            if not isinstance(sample, A2TelemetrySample):
                raise TypeError("sample must be an A2TelemetrySample")
            now = _integer(receive_time_ns, "receive_time_ns", positive=True)
            if not isinstance(rearm, bool):
                raise TypeError("rearm must be a boolean")
            if self._last_receive_time_ns is not None and now <= self._last_receive_time_ns:
                return self._failed(
                    A2TelemetryReason.ORDER,
                    "receive_time_ns must strictly increase",
                )
            self._last_receive_time_ns = now

            session_changed = (
                self._session_id is not None and sample.session_id != self._session_id
            )
            if self._restart_required:
                if not rearm:
                    return self._failed(
                        A2TelemetryReason.RESET_REQUIRED,
                        "telemetry stream requires an explicit rearm",
                    )
                if not session_changed:
                    return self._failed(
                        A2TelemetryReason.SESSION,
                        "rearm requires a new producer session",
                    )
            elif session_changed and not rearm:
                return self._failed(
                    A2TelemetryReason.SESSION,
                    "producer session changed without rearm",
                )

            if session_changed and sample.session_id in self._used_sessions:
                return self._failed(
                    A2TelemetryReason.SESSION,
                    "producer session was already retired",
                )

            if sample.publish_time_ns > now + self.config.max_future_skew_ns:
                return self._failed(
                    A2TelemetryReason.TIMING,
                    "publish timestamp is too far in the future",
                )
            if max(0, now - sample.publish_time_ns) > self.config.max_transport_age_ns:
                return self._failed(A2TelemetryReason.TIMING, "telemetry is stale")
            if max(0, now - sample.state_capture_time_ns) > self.config.max_state_age_ns:
                return self._failed(A2TelemetryReason.TIMING, "robot state is stale")
            if (
                max(0, now - sample.base_velocity_capture_time_ns)
                > self.config.max_velocity_age_ns
            ):
                return self._failed(A2TelemetryReason.TIMING, "base velocity is stale")
            if (
                abs(
                    sample.state_capture_time_ns
                    - sample.base_velocity_capture_time_ns
                )
                > self.config.max_state_velocity_skew_ns
            ):
                return self._failed(
                    A2TelemetryReason.TIMING,
                    "robot state and base velocity exceed the skew bound",
                )
            if not sample.motion_permitted:
                detail = f"controller phase={sample.phase}, fault={sample.fault or 'none'}"
                return self._failed(A2TelemetryReason.MOTION_BLOCKED, detail)

            same_session = self._session_id == sample.session_id
            if same_session and self._last_sequence is not None:
                if sample.sequence != self._last_sequence + 1:
                    return self._failed(
                        A2TelemetryReason.ORDER,
                        "telemetry sequence must be contiguous",
                    )
                if sample.state_sequence <= int(self._last_state_sequence):
                    return self._failed(
                        A2TelemetryReason.ORDER,
                        "robot-state sequence must strictly increase",
                    )
                if sample.state_capture_time_ns <= int(
                    self._last_state_capture_time_ns
                ):
                    return self._failed(
                        A2TelemetryReason.ORDER,
                        "state capture time must strictly increase",
                    )
                if sample.base_velocity_capture_time_ns <= int(
                    self._last_velocity_capture_time_ns
                ):
                    return self._failed(
                        A2TelemetryReason.ORDER,
                        "base-velocity capture time must strictly increase",
                    )
                if sample.publish_time_ns <= int(self._last_publish_time_ns):
                    return self._failed(
                        A2TelemetryReason.ORDER,
                        "publish time must strictly increase",
                    )

            if session_changed:
                assert self._session_id is not None
                self._used_sessions.add(self._session_id)
            self._session_id = sample.session_id
            self._last_sequence = sample.sequence
            self._last_state_sequence = sample.state_sequence
            self._last_state_capture_time_ns = sample.state_capture_time_ns
            self._last_velocity_capture_time_ns = (
                sample.base_velocity_capture_time_ns
            )
            self._last_publish_time_ns = sample.publish_time_ns
            self._restart_required = False
            return A2TelemetryDecision(
                True,
                False,
                A2TelemetryReason.ACCEPTED,
                "telemetry accepted",
                sample,
            )


class A2TelemetryPublisher:
    """Thread-safe loopback-only UDP publisher for controller-owned samples."""

    def __init__(
        self,
        port: int = A2_TELEMETRY_DEFAULT_PORT,
        *,
        host: str = "127.0.0.1",
        socket_factory: Callable[..., socket.socket] = socket.socket,
    ) -> None:
        address = ipaddress.ip_address(host)
        if address.version != 4 or not address.is_loopback:
            raise ValueError("telemetry destination must be IPv4 loopback")
        if not isinstance(port, int) or isinstance(port, bool) or not 1 <= port <= 65535:
            raise ValueError("telemetry port must be in [1, 65535]")
        if not callable(socket_factory):
            raise TypeError("socket_factory must be callable")
        self._destination = (str(address), port)
        self._socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        self._lock = threading.RLock()
        self._closed = False

    def publish(self, sample: A2TelemetrySample) -> None:
        if not isinstance(sample, A2TelemetrySample):
            raise TypeError("sample must be an A2TelemetrySample")
        payload = sample.encode()
        with self._lock:
            if self._closed:
                raise RuntimeError("telemetry publisher is closed")
            written = self._socket.sendto(payload, self._destination)
            if written != len(payload):
                raise OSError("short telemetry UDP send")

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._socket.close()

    def __enter__(self) -> "A2TelemetryPublisher":
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()


class A2TelemetryReceiver:
    """Non-blocking loopback receiver that validates every queued datagram."""

    def __init__(
        self,
        port: int = A2_TELEMETRY_DEFAULT_PORT,
        *,
        host: str = "127.0.0.1",
        max_datagrams_per_poll: int = 64,
        gate: A2TelemetryGate | None = None,
    ) -> None:
        address = ipaddress.ip_address(host)
        if address.version != 4 or not address.is_loopback:
            raise ValueError("telemetry receiver must bind IPv4 loopback")
        if not isinstance(port, int) or isinstance(port, bool) or not 0 <= port <= 65535:
            raise ValueError("telemetry port must be in [0, 65535]")
        if (
            not isinstance(max_datagrams_per_poll, int)
            or isinstance(max_datagrams_per_poll, bool)
            or max_datagrams_per_poll <= 0
        ):
            raise ValueError("max_datagrams_per_poll must be positive")
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.bind((str(address), port))
        self._socket.setblocking(False)
        self._port = int(self._socket.getsockname()[1])
        self._max_datagrams_per_poll = max_datagrams_per_poll
        self._gate = gate or A2TelemetryGate()
        self._lock = threading.RLock()
        self._closed = False

    @property
    def port(self) -> int:
        return self._port

    def receive_latest(
        self,
        receive_time_ns: int,
        *,
        rearm: bool = False,
    ) -> A2TelemetryDecision | None:
        with self._lock:
            if self._closed:
                return A2TelemetryDecision(
                    False,
                    True,
                    A2TelemetryReason.CLOSED,
                    "telemetry receiver is closed",
                    None,
                )
            datagrams: list[bytes] = []
            addresses: list[tuple[str, int]] = []
            for _ in range(self._max_datagrams_per_poll + 1):
                try:
                    payload, address = self._socket.recvfrom(
                        A2_TELEMETRY_MAX_DATAGRAM_BYTES + 1
                    )
                except BlockingIOError:
                    break
                datagrams.append(payload)
                addresses.append((str(address[0]), int(address[1])))
            if not datagrams:
                return self._gate.invalidate(
                    "no telemetry datagram is available",
                    reason=A2TelemetryReason.TIMING,
                )
            if len(datagrams) > self._max_datagrams_per_poll:
                while True:
                    try:
                        self._socket.recvfrom(A2_TELEMETRY_MAX_DATAGRAM_BYTES + 1)
                    except BlockingIOError:
                        break
                return self._gate.invalidate(
                    "telemetry UDP backlog exceeded the per-poll budget",
                    reason=A2TelemetryReason.BACKLOG,
                )

            decision: A2TelemetryDecision | None = None
            timestamp = _integer(
                receive_time_ns, "receive_time_ns", positive=True
            )
            for index, (payload, address) in enumerate(zip(datagrams, addresses)):
                try:
                    if not ipaddress.ip_address(address[0]).is_loopback:
                        raise A2TelemetryError("telemetry sender is not loopback")
                    sample = A2TelemetrySample.decode(payload)
                except A2TelemetryError as error:
                    return self._gate.invalidate(str(error))
                decision = self._gate.evaluate(
                    sample,
                    timestamp + index,
                    rearm=rearm and index == 0,
                )
                if not decision.accepted:
                    return decision
            return decision

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._socket.close()

    def __enter__(self) -> "A2TelemetryReceiver":
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        self.close()


__all__ = [
    "A2_LOW_LEVEL_CONTRACT_ID",
    "A2_LOW_LEVEL_POLICY_SHA256",
    "A2_TELEMETRY_CLOCK",
    "A2_TELEMETRY_DEFAULT_PORT",
    "A2_TELEMETRY_MAX_DATAGRAM_BYTES",
    "A2_TELEMETRY_SCHEMA",
    "A2TelemetryDecision",
    "A2TelemetryError",
    "A2TelemetryGate",
    "A2TelemetryGateConfig",
    "A2TelemetryPublisher",
    "A2TelemetryReason",
    "A2TelemetryReceiver",
    "A2TelemetrySample",
]

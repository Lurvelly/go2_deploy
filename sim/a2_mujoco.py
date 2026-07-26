#!/usr/bin/env python3
"""Standalone MuJoCo validation runner for an A2 45D policy contract.

This validates the policy/physics contract. It deliberately does not emulate
the DDS, MotionSwitcher, CRC, or mainboard safety gates used on the real robot.
"""

import argparse
import hashlib
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml


EXPECTED_CONTRACT_ID = "a2_45d_project_v0"
EXPECTED_JOINT_NAMES = (
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
)
EXPECTED_DEFAULT_POSITIONS = np.asarray(
    [0.1, 0.9, -1.8, -0.1, 0.9, -1.8, 0.1, 0.9, -1.8, -0.1, 0.9, -1.8],
    dtype=np.float64,
)


class ContractError(RuntimeError):
    """The configured A2 policy/robot contract is internally inconsistent."""


class SimulationFault(RuntimeError):
    """A fail-closed runtime condition occurred in simulation."""


@dataclass(frozen=True)
class Contract:
    policy_path: Path
    policy_sha256: str
    policy_golden_output: Optional[np.ndarray]
    observation_dim: int
    action_dim: int
    policy_dt: float
    action_scale: float
    command_scale: np.ndarray
    angular_velocity_scale: float
    joint_position_scale: float
    joint_velocity_scale: float
    joint_names: Tuple[str, ...]
    default_positions: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    lower: np.ndarray
    upper: np.ndarray
    velocity: np.ndarray
    effort: np.ndarray
    command_bounds: Dict[str, Tuple[float, float]]
    max_tilt_rad: float
    quaternion_norm_min: float
    quaternion_norm_max: float
    soft_position_limit_ratio: float
    xml_path: Path
    physics_dt: float
    base_height: float
    joint_position_tolerance_rad: float
    policy_decimation: int


@dataclass(frozen=True)
class SimulationSummary:
    simulated_seconds: float
    steps: int
    policy_steps: int
    max_abs_torque: float
    max_abs_joint_velocity: float
    max_tilt_rad: float


def _mapping(parent: Mapping[str, Any], key: str, path: str) -> Mapping[str, Any]:
    value = parent.get(key)
    if not isinstance(value, Mapping):
        raise ContractError("{} must be a mapping".format(path))
    return value


def _integer(parent: Mapping[str, Any], key: str, path: str) -> int:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ContractError("{} must be an integer".format(path))
    return int(value)


def _number(
    parent: Mapping[str, Any], key: str, path: str, *, positive: bool = False
) -> float:
    value = parent.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ContractError("{} must be a number".format(path))
    result = float(value)
    if not math.isfinite(result):
        raise ContractError("{} must be finite".format(path))
    if positive and result <= 0.0:
        raise ContractError("{} must be positive".format(path))
    return result


def _vector(
    parent: Mapping[str, Any], key: str, path: str, length: int
) -> np.ndarray:
    value = parent.get(key)
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise ContractError("{} must be a {}-element sequence".format(path, length))
    if len(value) != length:
        raise ContractError(
            "{} has length {}, expected {}".format(path, len(value), length)
        )
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ContractError("{} must contain only numbers".format(path)) from exc
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ContractError("{} must contain {} finite numbers".format(path, length))
    return result


def _project_path(project_root: Path, raw_path: Any, path: str) -> Path:
    if not isinstance(raw_path, str) or not raw_path:
        raise ContractError("{} must be a non-empty path string".format(path))
    candidate = Path(raw_path).expanduser()
    if not candidate.is_absolute():
        candidate = project_root / candidate
    candidate = candidate.resolve()
    if not candidate.is_file():
        raise ContractError("{} does not exist: {}".format(path, candidate))
    return candidate


def _require_close(actual: float, expected: float, path: str) -> None:
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=1.0e-12):
        raise ContractError("{} is {}, expected {}".format(path, actual, expected))


def _integer_ratio(numerator: float, denominator: float, path: str) -> int:
    ratio = numerator / denominator
    rounded = int(round(ratio))
    if rounded < 1 or not math.isclose(ratio, rounded, rel_tol=0.0, abs_tol=1.0e-9):
        raise ContractError("{} must be a positive integer, got {}".format(path, ratio))
    return rounded


def load_contract(config_path: Path, project_root: Path) -> Contract:
    """Load and strictly validate the shared A2 YAML contract."""

    try:
        with config_path.open("r", encoding="utf-8") as stream:
            root = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise ContractError("cannot read config {}: {}".format(config_path, exc)) from exc
    if not isinstance(root, Mapping):
        raise ContractError("configuration root must be a mapping")

    if _integer(root, "config_version", "config_version") != 1:
        raise ContractError("config_version must be 1")
    if root.get("contract_id") != EXPECTED_CONTRACT_ID:
        raise ContractError(
            "contract_id must be {!r}".format(EXPECTED_CONTRACT_ID)
        )

    policy = _mapping(root, "policy", "policy")
    policy_path = _project_path(project_root, policy.get("file"), "policy.file")
    policy_sha256 = policy.get("sha256")
    if (
        not isinstance(policy_sha256, str)
        or len(policy_sha256) != 64
        or any(value not in "0123456789abcdef" for value in policy_sha256)
    ):
        raise ContractError(
            "policy.sha256 must be a 64-character lowercase hex digest"
        )
    golden_raw = policy.get("golden_output")
    policy_golden_output = (
        None
        if golden_raw is None
        else _vector(policy, "golden_output", "policy.golden_output", 12)
    )
    observation_dim = _integer(policy, "observation_dim", "policy.observation_dim")
    action_dim = _integer(policy, "action_dim", "policy.action_dim")
    if observation_dim != 45 or action_dim != 12:
        raise ContractError("the A2 45D ABI requires observation_dim=45, action_dim=12")
    policy_dt = _number(policy, "dt", "policy.dt", positive=True)
    action_scale = _number(policy, "action_scale", "policy.action_scale", positive=True)
    _require_close(policy_dt, 0.02, "policy.dt")
    _require_close(action_scale, 0.25, "policy.action_scale")

    scales = _mapping(policy, "scales", "policy.scales")
    command_scale = _vector(scales, "command", "policy.scales.command", 3)
    angular_velocity_scale = _number(
        scales, "angular_velocity", "policy.scales.angular_velocity"
    )
    joint_position_scale = _number(
        scales, "joint_position", "policy.scales.joint_position"
    )
    joint_velocity_scale = _number(
        scales, "joint_velocity", "policy.scales.joint_velocity"
    )
    if not np.array_equal(command_scale, np.asarray([1.0, 1.0, 0.25])):
        raise ContractError("policy.scales.command must be [1.0, 1.0, 0.25]")
    _require_close(angular_velocity_scale, 0.25, "policy.scales.angular_velocity")
    _require_close(joint_position_scale, 1.0, "policy.scales.joint_position")
    _require_close(joint_velocity_scale, 0.05, "policy.scales.joint_velocity")

    runtime = _mapping(root, "runtime", "runtime")
    if _integer(runtime, "motor_slots", "runtime.motor_slots") != 35:
        raise ContractError("runtime.motor_slots must be 35 for HG LowCmd/LowState")
    command_dt = _number(runtime, "command_dt", "runtime.command_dt", positive=True)
    _require_close(command_dt, 0.002, "runtime.command_dt")
    _integer_ratio(policy_dt, command_dt, "policy.dt / runtime.command_dt")

    robot = _mapping(root, "robot", "robot")
    joint_names_raw = robot.get("joint_names")
    if not isinstance(joint_names_raw, Sequence) or isinstance(
        joint_names_raw, (str, bytes)
    ):
        raise ContractError("robot.joint_names must be a sequence")
    joint_names = tuple(joint_names_raw)
    if joint_names != EXPECTED_JOINT_NAMES:
        raise ContractError(
            "robot.joint_names must use FR, FL, RR, RL x hip, thigh, calf order"
        )
    motor_indices = _vector(robot, "motor_indices", "robot.motor_indices", action_dim)
    if not np.array_equal(motor_indices, np.arange(action_dim, dtype=np.float64)):
        raise ContractError("robot.motor_indices must be the first 12 HG motor slots")
    default_positions = _vector(
        robot, "default_positions", "robot.default_positions", action_dim
    )
    if not np.array_equal(default_positions, EXPECTED_DEFAULT_POSITIONS):
        raise ContractError("robot.default_positions does not match the policy ABI")
    kp = _vector(robot, "kp", "robot.kp", action_dim)
    kd = _vector(robot, "kd", "robot.kd", action_dim)
    if np.any(kp <= 0.0) or np.any(kd < 0.0):
        raise ContractError("robot.kp must be positive and robot.kd non-negative")

    limits = _mapping(robot, "limits", "robot.limits")
    lower = _vector(limits, "lower", "robot.limits.lower", action_dim)
    upper = _vector(limits, "upper", "robot.limits.upper", action_dim)
    velocity = _vector(limits, "velocity", "robot.limits.velocity", action_dim)
    effort = _vector(limits, "effort", "robot.limits.effort", action_dim)
    if np.any(lower >= upper):
        raise ContractError("every hard joint lower limit must be below its upper limit")
    if np.any(default_positions < lower) or np.any(default_positions > upper):
        raise ContractError("robot.default_positions must lie inside hard limits")
    if np.any(velocity <= 0.0) or np.any(effort <= 0.0):
        raise ContractError("joint velocity and effort limits must be positive")

    commands = _mapping(root, "commands", "commands")
    command_bounds = {}
    for key in ("vx", "vy", "yaw"):
        bound = _vector(commands, key, "commands.{}".format(key), 2)
        if bound[0] > bound[1]:
            raise ContractError("commands.{} lower bound exceeds upper bound".format(key))
        command_bounds[key] = (float(bound[0]), float(bound[1]))

    safety = _mapping(root, "safety", "safety")
    for key in (
        "low_state_timeout_ms",
        "initial_state_timeout_ms",
        "mainboard_prearm_timeout_ms",
        "mainboard_runtime_timeout_ms",
        "mode_release_timeout_ms",
    ):
        if _integer(safety, key, "safety.{}".format(key)) <= 0:
            raise ContractError("safety.{} must be positive".format(key))
    max_tilt_rad = _number(safety, "max_tilt_rad", "safety.max_tilt_rad", positive=True)
    quaternion_norm_min = _number(
        safety, "quaternion_norm_min", "safety.quaternion_norm_min", positive=True
    )
    quaternion_norm_max = _number(
        safety, "quaternion_norm_max", "safety.quaternion_norm_max", positive=True
    )
    soft_position_limit_ratio = _number(
        safety,
        "soft_position_limit_ratio",
        "safety.soft_position_limit_ratio",
        positive=True,
    )
    if max_tilt_rad > math.pi:
        raise ContractError("safety.max_tilt_rad must not exceed pi")
    if quaternion_norm_min >= quaternion_norm_max:
        raise ContractError("quaternion norm bounds are inverted")
    if soft_position_limit_ratio > 1.0:
        raise ContractError("safety.soft_position_limit_ratio must not exceed 1")
    _require_close(max_tilt_rad, 1.0, "safety.max_tilt_rad")
    _require_close(quaternion_norm_min, 0.9, "safety.quaternion_norm_min")
    _require_close(quaternion_norm_max, 1.1, "safety.quaternion_norm_max")
    _require_close(
        soft_position_limit_ratio, 0.9, "safety.soft_position_limit_ratio"
    )
    for key in ("stand_duration_s", "damping_duration_s", "stop_duration_s"):
        _number(safety, key, "safety.{}".format(key), positive=True)

    sim = _mapping(root, "sim", "sim")
    xml_path = _project_path(project_root, sim.get("xml"), "sim.xml")
    physics_dt = _number(sim, "physics_dt", "sim.physics_dt", positive=True)
    base_height = _number(sim, "base_height", "sim.base_height", positive=True)
    joint_position_tolerance_rad = _number(
        sim,
        "joint_position_tolerance_rad",
        "sim.joint_position_tolerance_rad",
    )
    if not 0.0 <= joint_position_tolerance_rad <= 0.01:
        raise ContractError(
            "sim.joint_position_tolerance_rad must be in [0, 0.01]"
        )
    policy_decimation = _integer_ratio(
        policy_dt, physics_dt, "policy.dt / sim.physics_dt"
    )

    return Contract(
        policy_path=policy_path,
        policy_sha256=policy_sha256,
        policy_golden_output=policy_golden_output,
        observation_dim=observation_dim,
        action_dim=action_dim,
        policy_dt=policy_dt,
        action_scale=action_scale,
        command_scale=command_scale,
        angular_velocity_scale=angular_velocity_scale,
        joint_position_scale=joint_position_scale,
        joint_velocity_scale=joint_velocity_scale,
        joint_names=joint_names,
        default_positions=default_positions,
        kp=kp,
        kd=kd,
        lower=lower,
        upper=upper,
        velocity=velocity,
        effort=effort,
        command_bounds=command_bounds,
        max_tilt_rad=max_tilt_rad,
        quaternion_norm_min=quaternion_norm_min,
        quaternion_norm_max=quaternion_norm_max,
        soft_position_limit_ratio=soft_position_limit_ratio,
        xml_path=xml_path,
        physics_dt=physics_dt,
        base_height=base_height,
        joint_position_tolerance_rad=joint_position_tolerance_rad,
        policy_decimation=policy_decimation,
    )


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_policy(contract: Contract):
    """Hash, load, and probe the TorchScript policy on CPU."""

    actual_hash = sha256_file(contract.policy_path)
    if actual_hash != contract.policy_sha256:
        raise ContractError(
            "policy hash mismatch: expected {}, got {}".format(
                contract.policy_sha256, actual_hash
            )
        )
    try:
        import torch
    except ImportError as exc:
        raise ContractError("PyTorch is required to load the A2 policy") from exc
    try:
        policy = torch.jit.load(str(contract.policy_path), map_location="cpu")
        policy.eval()
        probe = torch.zeros((1, contract.observation_dim), dtype=torch.float32)
        probe[0, 5] = -1.0
        with torch.inference_mode():
            output = policy(probe)
    except Exception as exc:
        raise ContractError("cannot load/probe policy: {}".format(exc)) from exc
    if not isinstance(output, torch.Tensor) or tuple(output.shape) != (
        1,
        contract.action_dim,
    ):
        shape = getattr(output, "shape", None)
        raise ContractError(
            "policy must map (1, 45) to (1, 12), got {!r}".format(shape)
        )
    if output.dtype != torch.float32:
        raise ContractError("policy output must use float32, got {}".format(output.dtype))
    if not bool(torch.isfinite(output).all()):
        raise ContractError("policy shape probe returned a non-finite action")
    if contract.policy_golden_output is not None:
        actual = output.detach().cpu().numpy()[0].astype(np.float64)
        if not np.allclose(
            actual, contract.policy_golden_output, rtol=0.0, atol=5.0e-5
        ):
            index = int(np.argmax(np.abs(actual - contract.policy_golden_output)))
            raise ContractError(
                "policy golden-output mismatch at index {}: expected {}, got {}".format(
                    index, contract.policy_golden_output[index], actual[index]
                )
            )
    return torch, policy


def projected_gravity(quaternion_wxyz: np.ndarray) -> np.ndarray:
    """Return the unit world gravity vector expressed in the base frame."""

    quaternion = np.asarray(quaternion_wxyz, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 0.0:
        raise SimulationFault("base quaternion is invalid")
    qw, qx, qy, qz = quaternion / norm
    return np.asarray(
        [
            2.0 * (qw * qy - qz * qx),
            -2.0 * (qz * qy + qw * qx),
            1.0 - 2.0 * (qw * qw + qz * qz),
        ],
        dtype=np.float32,
    )


def soft_position_bounds(contract: Contract) -> Tuple[np.ndarray, np.ndarray]:
    center = 0.5 * (contract.lower + contract.upper)
    half_range = 0.5 * (contract.upper - contract.lower)
    scaled_half_range = contract.soft_position_limit_ratio * half_range
    return center - scaled_half_range, center + scaled_half_range


def build_observation(
    contract: Contract,
    command: np.ndarray,
    quaternion: np.ndarray,
    body_angular_velocity: np.ndarray,
    joint_position: np.ndarray,
    joint_velocity: np.ndarray,
    previous_raw_action: np.ndarray,
) -> np.ndarray:
    observation = np.concatenate(
        (
            command * contract.command_scale,
            projected_gravity(quaternion),
            body_angular_velocity * contract.angular_velocity_scale,
            (joint_position - contract.default_positions)
            * contract.joint_position_scale,
            joint_velocity * contract.joint_velocity_scale,
            previous_raw_action,
        )
    ).astype(np.float32, copy=False)
    if observation.shape != (contract.observation_dim,):
        raise SimulationFault(
            "observation shape is {}, expected ({},)".format(
                observation.shape, contract.observation_dim
            )
        )
    if not np.all(np.isfinite(observation)):
        raise SimulationFault("observation contains a non-finite value")
    return observation


def _joint_addresses(mujoco, model, joint_names: Sequence[str]):
    qpos_addresses = []
    qvel_addresses = []
    joint_ids = []
    for name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            raise ContractError("MJCF is missing policy joint {}".format(name))
        if int(model.jnt_type[joint_id]) != int(mujoco.mjtJoint.mjJNT_HINGE):
            raise ContractError("MJCF joint {} is not a hinge".format(name))
        joint_ids.append(joint_id)
        qpos_addresses.append(int(model.jnt_qposadr[joint_id]))
        qvel_addresses.append(int(model.jnt_dofadr[joint_id]))
    return (
        np.asarray(joint_ids, dtype=np.int32),
        np.asarray(qpos_addresses, dtype=np.int32),
        np.asarray(qvel_addresses, dtype=np.int32),
    )


def _actuator_ids(mujoco, model, joint_ids: np.ndarray) -> np.ndarray:
    result = []
    for joint_id in joint_ids:
        matches = [
            actuator_id
            for actuator_id in range(model.nu)
            if int(model.actuator_trnid[actuator_id, 0]) == int(joint_id)
        ]
        if len(matches) != 1:
            raise ContractError(
                "joint id {} must have exactly one actuator, found {}".format(
                    int(joint_id), len(matches)
                )
            )
        result.append(matches[0])
    return np.asarray(result, dtype=np.int32)


def _sensor_slice(mujoco, model, sensor_name: str) -> slice:
    sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
    if sensor_id < 0:
        raise ContractError("MJCF is missing sensor {}".format(sensor_name))
    address = int(model.sensor_adr[sensor_id])
    dimension = int(model.sensor_dim[sensor_id])
    if dimension != 3:
        raise ContractError("sensor {} must be three-dimensional".format(sensor_name))
    return slice(address, address + dimension)


def _validate_model_contract(
    contract: Contract, model, joint_ids: np.ndarray, actuator_ids: np.ndarray
) -> None:
    model_ranges = np.asarray(model.jnt_range[joint_ids], dtype=np.float64)
    configured_ranges = np.column_stack((contract.lower, contract.upper))
    if not np.all(np.asarray(model.jnt_limited[joint_ids], dtype=bool)):
        raise ContractError("all 12 A2 policy joints must be range-limited")
    if not np.allclose(model_ranges, configured_ranges, rtol=0.0, atol=1.0e-6):
        raise ContractError("MJCF joint ranges do not match robot.limits")
    if not np.all(np.asarray(model.actuator_ctrllimited[actuator_ids], dtype=bool)):
        raise ContractError("all 12 A2 actuators must have control limits")
    actuator_ranges = np.asarray(model.actuator_ctrlrange[actuator_ids], dtype=np.float64)
    configured_effort = np.column_stack((-contract.effort, contract.effort))
    if not np.allclose(actuator_ranges, configured_effort, rtol=0.0, atol=1.0e-6):
        raise ContractError("MJCF actuator ranges do not match robot effort limits")


def _validate_command(contract: Contract, vx: float, vy: float, yaw: float) -> np.ndarray:
    values = {"vx": vx, "vy": vy, "yaw": yaw}
    for name, value in values.items():
        if not math.isfinite(value):
            raise ContractError("command {} must be finite".format(name))
        lower, upper = contract.command_bounds[name]
        if value < lower or value > upper:
            raise ContractError(
                "command {}={} is outside [{}, {}]".format(name, value, lower, upper)
            )
    return np.asarray([vx, vy, yaw], dtype=np.float64)


def _state_metrics(
    contract: Contract,
    data,
    base_quaternion: np.ndarray,
    joint_position: np.ndarray,
    joint_velocity: np.ndarray,
) -> float:
    if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
        raise SimulationFault("MuJoCo state contains a non-finite value")
    quaternion_norm = float(np.linalg.norm(base_quaternion))
    if (
        quaternion_norm < contract.quaternion_norm_min
        or quaternion_norm > contract.quaternion_norm_max
    ):
        raise SimulationFault(
            "base quaternion norm {:.6g} is outside [{}, {}]".format(
                quaternion_norm,
                contract.quaternion_norm_min,
                contract.quaternion_norm_max,
            )
        )
    outside = np.flatnonzero(
        (joint_position < contract.lower - contract.joint_position_tolerance_rad)
        | (joint_position > contract.upper + contract.joint_position_tolerance_rad)
    )
    if outside.size:
        index = int(outside[0])
        raise SimulationFault(
            "{} position {:.6g} is outside hard limits [{}, {}]".format(
                contract.joint_names[index],
                joint_position[index],
                contract.lower[index],
                contract.upper[index],
            )
        )
    velocity_outside = np.flatnonzero(
        np.abs(joint_velocity) > contract.velocity + 1.0e-6
    )
    if velocity_outside.size:
        index = int(velocity_outside[0])
        raise SimulationFault(
            "{} velocity {:.6g} exceeds +/-{}".format(
                contract.joint_names[index],
                joint_velocity[index],
                contract.velocity[index],
            )
        )
    gravity = projected_gravity(base_quaternion)
    tilt = math.acos(float(np.clip(-gravity[2], -1.0, 1.0)))
    if tilt > contract.max_tilt_rad:
        raise SimulationFault(
            "base tilt {:.6g} rad exceeds {} rad".format(tilt, contract.max_tilt_rad)
        )
    return tilt


def run_simulation(
    contract: Contract,
    torch,
    policy,
    command: np.ndarray,
    duration: float,
    headless: bool,
) -> SimulationSummary:
    try:
        import mujoco
    except ImportError as exc:
        raise ContractError("the mujoco Python package is required") from exc

    try:
        model = mujoco.MjModel.from_xml_path(str(contract.xml_path))
    except Exception as exc:
        raise ContractError("cannot load MJCF {}: {}".format(contract.xml_path, exc)) from exc
    model.opt.timestep = contract.physics_dt
    data = mujoco.MjData(model)

    joint_ids, qpos_addresses, qvel_addresses = _joint_addresses(
        mujoco, model, contract.joint_names
    )
    actuator_ids = _actuator_ids(mujoco, model, joint_ids)
    _validate_model_contract(contract, model, joint_ids, actuator_ids)
    gyro_slice = _sensor_slice(mujoco, model, "imu_gyro")

    free_joint_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint"
    )
    if free_joint_id < 0 or int(model.jnt_type[free_joint_id]) != int(
        mujoco.mjtJoint.mjJNT_FREE
    ):
        raise ContractError("MJCF must contain floating_base_joint as a free joint")
    base_qpos_address = int(model.jnt_qposadr[free_joint_id])

    data.qpos[:] = model.qpos0
    data.qpos[base_qpos_address : base_qpos_address + 3] = np.asarray(
        [0.0, 0.0, contract.base_height]
    )
    data.qpos[base_qpos_address + 3 : base_qpos_address + 7] = np.asarray(
        [1.0, 0.0, 0.0, 0.0]
    )
    data.qpos[qpos_addresses] = contract.default_positions
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)

    soft_lower, soft_upper = soft_position_bounds(contract)
    previous_raw_action = np.zeros(contract.action_dim, dtype=np.float64)
    desired_target = contract.default_positions.copy()
    target = contract.default_positions.copy()
    maximum_target_step = contract.velocity * contract.physics_dt
    step_count = 0
    policy_step_count = 0
    max_abs_torque = 0.0
    max_abs_joint_velocity = 0.0
    max_tilt = 0.0

    def advance(viewer=None) -> None:
        nonlocal desired_target
        nonlocal max_abs_joint_velocity
        nonlocal max_abs_torque
        nonlocal max_tilt
        nonlocal policy_step_count
        nonlocal previous_raw_action
        nonlocal step_count
        nonlocal target

        wall_start = time.monotonic()
        while data.time + 0.5 * contract.physics_dt < duration:
            if viewer is not None and not viewer.is_running():
                break
            step_wall_start = time.monotonic()
            joint_position = np.asarray(data.qpos[qpos_addresses], dtype=np.float64)
            joint_velocity = np.asarray(data.qvel[qvel_addresses], dtype=np.float64)
            quaternion = np.asarray(
                data.qpos[base_qpos_address + 3 : base_qpos_address + 7],
                dtype=np.float64,
            )
            max_tilt = max(
                max_tilt,
                _state_metrics(
                    contract,
                    data,
                    quaternion,
                    joint_position,
                    joint_velocity,
                ),
            )

            target_delta = np.clip(
                desired_target - target, -maximum_target_step, maximum_target_step
            )
            target = np.clip(target + target_delta, soft_lower, soft_upper)
            torque = contract.kp * (target - joint_position) - contract.kd * joint_velocity
            if not np.all(np.isfinite(torque)):
                raise SimulationFault("PD controller produced a non-finite torque")
            torque = np.clip(torque, -contract.effort, contract.effort)
            data.ctrl[:] = 0.0
            data.ctrl[actuator_ids] = torque
            max_abs_torque = max(max_abs_torque, float(np.max(np.abs(torque))))
            max_abs_joint_velocity = max(
                max_abs_joint_velocity, float(np.max(np.abs(joint_velocity)))
            )

            mujoco.mj_step(model, data)
            step_count += 1

            joint_position = np.asarray(data.qpos[qpos_addresses], dtype=np.float64)
            joint_velocity = np.asarray(data.qvel[qvel_addresses], dtype=np.float64)
            quaternion = np.asarray(
                data.qpos[base_qpos_address + 3 : base_qpos_address + 7],
                dtype=np.float64,
            )
            max_tilt = max(
                max_tilt,
                _state_metrics(
                    contract,
                    data,
                    quaternion,
                    joint_position,
                    joint_velocity,
                ),
            )

            if step_count % contract.policy_decimation == 0:
                body_angular_velocity = np.asarray(
                    data.sensordata[gyro_slice], dtype=np.float64
                )
                observation = build_observation(
                    contract,
                    command,
                    quaternion,
                    body_angular_velocity,
                    joint_position,
                    joint_velocity,
                    previous_raw_action,
                )
                with torch.inference_mode():
                    output = policy(torch.from_numpy(observation).unsqueeze(0))
                if not isinstance(output, torch.Tensor) or tuple(output.shape) != (
                    1,
                    contract.action_dim,
                ):
                    raise SimulationFault("policy output shape changed during simulation")
                raw_action = output.detach().cpu().numpy()[0].astype(np.float64)
                if not np.all(np.isfinite(raw_action)):
                    raise SimulationFault("policy produced a non-finite action")
                previous_raw_action = raw_action
                desired_target = np.clip(
                    contract.default_positions + contract.action_scale * raw_action,
                    soft_lower,
                    soft_upper,
                )
                policy_step_count += 1

            if viewer is not None:
                viewer.sync()
                remaining = contract.physics_dt - (time.monotonic() - step_wall_start)
                if remaining > 0.0:
                    time.sleep(remaining)

        if viewer is not None:
            elapsed = time.monotonic() - wall_start
            if elapsed <= 0.0:
                raise SimulationFault("viewer loop did not advance")

    if headless:
        advance()
    else:
        try:
            import mujoco.viewer
        except ImportError as exc:
            raise ContractError("mujoco.viewer is unavailable") from exc
        with mujoco.viewer.launch_passive(model, data) as viewer:
            advance(viewer)

    if headless and step_count == 0:
        raise SimulationFault("headless simulation did not advance")
    return SimulationSummary(
        simulated_seconds=float(data.time),
        steps=step_count,
        policy_steps=policy_step_count,
        max_abs_torque=max_abs_torque,
        max_abs_joint_velocity=max_abs_joint_velocity,
        max_tilt_rad=max_tilt,
    )


def _parse_args(argv=None):
    project_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(
        description="Run a configured A2 45D low policy in MuJoCo"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=project_root / "params" / "a2.yaml",
        help="shared A2 YAML contract (default: params/a2.yaml)",
    )
    parser.add_argument("--headless", action="store_true", help="run without a viewer")
    parser.add_argument(
        "--duration", type=float, default=5.0, help="simulated duration in seconds"
    )
    parser.add_argument("--vx", type=float, default=0.0, help="forward velocity command")
    parser.add_argument("--vy", type=float, default=0.0, help="lateral velocity command")
    parser.add_argument("--yaw", type=float, default=0.0, help="yaw-rate command")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv)
    project_root = Path(__file__).resolve().parents[1]
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        print("[a2_mujoco] fault: --duration must be positive and finite", file=sys.stderr)
        return 1
    try:
        contract = load_contract(args.config.expanduser().resolve(), project_root)
        command = _validate_command(contract, args.vx, args.vy, args.yaw)
        torch, policy = load_policy(contract)
        summary = run_simulation(
            contract, torch, policy, command, args.duration, args.headless
        )
    except (ContractError, SimulationFault) as exc:
        print("[a2_mujoco] fault: {}".format(exc), file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("[a2_mujoco] interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(
            "[a2_mujoco] unexpected fault: {}: {}".format(type(exc).__name__, exc),
            file=sys.stderr,
        )
        return 1

    print(
        "[a2_mujoco] ok: time={:.3f}s steps={} policy_steps={} "
        "max|tau|={:.3f}Nm max|dq|={:.3f}rad/s max_tilt={:.3f}rad".format(
            summary.simulated_seconds,
            summary.steps,
            summary.policy_steps,
            summary.max_abs_torque,
            summary.max_abs_joint_velocity,
            summary.max_tilt_rad,
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())

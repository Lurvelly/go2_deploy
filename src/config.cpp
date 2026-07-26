#include "a2/config.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>

#include <yaml-cpp/yaml.h>

namespace a2 {
namespace {

[[noreturn]] void ConfigError(const std::string& message) {
  throw std::runtime_error("invalid A2 configuration: " + message);
}

YAML::Node Required(const YAML::Node& parent, const char* key,
                    const std::string& context) {
  if (!parent || !parent.IsMap()) {
    ConfigError(context + " must be a mapping");
  }
  const YAML::Node value = parent[key];
  if (!value || !value.IsDefined()) {
    ConfigError("missing " + context + "." + key);
  }
  return value;
}

YAML::Node RequiredMap(const YAML::Node& parent, const char* key,
                       const std::string& context) {
  const YAML::Node value = Required(parent, key, context);
  if (!value.IsMap()) {
    ConfigError(context + "." + key + " must be a mapping");
  }
  return value;
}

template <typename T>
T Scalar(const YAML::Node& node, const std::string& context) {
  if (!node.IsScalar()) {
    ConfigError(context + " must be a scalar");
  }
  try {
    return node.as<T>();
  } catch (const YAML::Exception& error) {
    ConfigError(context + ": " + error.what());
  }
}

float FiniteFloat(const YAML::Node& node, const std::string& context) {
  const float value = Scalar<float>(node, context);
  if (!std::isfinite(value)) {
    ConfigError(context + " must be finite");
  }
  return value;
}

std::size_t NonNegativeSize(const YAML::Node& node,
                            const std::string& context) {
  const long long value = Scalar<long long>(node, context);
  if (value < 0) {
    ConfigError(context + " must not be negative");
  }
  return static_cast<std::size_t>(value);
}

std::chrono::milliseconds PositiveMilliseconds(const YAML::Node& node,
                                                const std::string& context) {
  const long long value = Scalar<long long>(node, context);
  if (value <= 0) {
    ConfigError(context + " must be positive");
  }
  return std::chrono::milliseconds(value);
}

std::chrono::milliseconds PositiveSeconds(const YAML::Node& node,
                                           const std::string& context) {
  const float seconds = FiniteFloat(node, context);
  if (seconds <= 0.0F) {
    ConfigError(context + " must be positive");
  }
  const auto milliseconds = static_cast<long long>(std::llround(seconds * 1000.0));
  if (milliseconds <= 0) {
    ConfigError(context + " is below one millisecond");
  }
  return std::chrono::milliseconds(milliseconds);
}

template <std::size_t N>
std::array<float, N> FloatArray(const YAML::Node& node,
                                const std::string& context) {
  if (!node.IsSequence() || node.size() != N) {
    ConfigError(context + " must contain exactly " + std::to_string(N) +
                " elements");
  }
  std::array<float, N> result{};
  for (std::size_t i = 0; i < N; ++i) {
    result[i] = FiniteFloat(node[i], context + "[" + std::to_string(i) + "]");
  }
  return result;
}

template <std::size_t N>
std::array<std::size_t, N> SizeArray(const YAML::Node& node,
                                     const std::string& context) {
  if (!node.IsSequence() || node.size() != N) {
    ConfigError(context + " must contain exactly " + std::to_string(N) +
                " elements");
  }
  std::array<std::size_t, N> result{};
  for (std::size_t i = 0; i < N; ++i) {
    result[i] =
        NonNegativeSize(node[i], context + "[" + std::to_string(i) + "]");
  }
  return result;
}

template <std::size_t N>
std::array<std::string, N> StringArray(const YAML::Node& node,
                                       const std::string& context) {
  if (!node.IsSequence() || node.size() != N) {
    ConfigError(context + " must contain exactly " + std::to_string(N) +
                " elements");
  }
  std::array<std::string, N> result{};
  for (std::size_t i = 0; i < N; ++i) {
    result[i] = Scalar<std::string>(
        node[i], context + "[" + std::to_string(i) + "]");
    if (result[i].empty()) {
      ConfigError(context + " contains an empty name");
    }
  }
  return result;
}

std::filesystem::path ResolveProjectPath(
    const std::filesystem::path& source_path, const std::string& configured) {
  if (configured.empty()) {
    ConfigError("configured file path must not be empty");
  }
  std::filesystem::path path(configured);
  if (path.is_absolute()) {
    return path.lexically_normal();
  }

  // params/a2.yaml deliberately stores project-root-relative asset paths so
  // C++ and the Python simulator consume the same strings.
  std::filesystem::path base = source_path.parent_path();
  if (base.filename() == "params") {
    base = base.parent_path();
  }
  return (base / path).lexically_normal();
}

bool NearlyEqual(const float lhs, const float rhs,
                 const float tolerance = 1.0e-5F) noexcept {
  return std::fabs(lhs - rhs) <= tolerance;
}

template <std::size_t N>
void RequireArrayEqual(const std::array<float, N>& actual,
                       const std::array<float, N>& expected,
                       const std::string& name) {
  for (std::size_t i = 0; i < N; ++i) {
    if (!NearlyEqual(actual[i], expected[i])) {
      ConfigError(name + " does not match the frozen A2 contract at index " +
                  std::to_string(i));
    }
  }
}

}  // namespace

A2Config::A2Config() {
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    joint_names[i] = std::string(kJointNames[i]);
    motor_indices[i] = i;
  }
  const auto soft_limits = ComputeSoftPositionLimits(
      hard_lower, hard_upper, soft_position_limit_ratio);
  soft_lower = soft_limits.first;
  soft_upper = soft_limits.second;
}

A2Config A2Config::Load(const std::filesystem::path& filename) {
  if (filename.empty()) {
    ConfigError("configuration filename must not be empty");
  }

  A2Config config;
  try {
    config.source_path = std::filesystem::absolute(filename).lexically_normal();
  } catch (const std::filesystem::filesystem_error& error) {
    ConfigError("cannot resolve configuration path: " +
                std::string(error.what()));
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(config.source_path.string());
  } catch (const YAML::Exception& error) {
    ConfigError("cannot load " + config.source_path.string() + ": " +
                error.what());
  }
  if (!root.IsMap()) {
    ConfigError("document root must be a mapping");
  }

  config.config_version =
      NonNegativeSize(Required(root, "config_version", "root"),
                      "config_version");
  config.contract_id =
      Scalar<std::string>(Required(root, "contract_id", "root"),
                          "contract_id");

  const YAML::Node policy = RequiredMap(root, "policy", "root");
  config.model_path = ResolveProjectPath(
      config.source_path,
      Scalar<std::string>(Required(policy, "file", "policy"), "policy.file"));
  config.policy_sha256 = Scalar<std::string>(
      Required(policy, "sha256", "policy"), "policy.sha256");
  const YAML::Node golden_output = policy["golden_output"];
  if (golden_output && golden_output.IsDefined() &&
      !golden_output.IsNull()) {
    config.policy_golden_output =
        FloatArray<kActionDim>(golden_output, "policy.golden_output");
  }
  config.observation_dim = NonNegativeSize(
      Required(policy, "observation_dim", "policy"),
      "policy.observation_dim");
  config.action_dim = NonNegativeSize(
      Required(policy, "action_dim", "policy"), "policy.action_dim");
  config.policy_dt =
      FiniteFloat(Required(policy, "dt", "policy"), "policy.dt");
  config.action_scale = FiniteFloat(
      Required(policy, "action_scale", "policy"), "policy.action_scale");

  const YAML::Node scales = RequiredMap(policy, "scales", "policy");
  config.command_scale = FloatArray<3>(
      Required(scales, "command", "policy.scales"),
      "policy.scales.command");
  config.angular_velocity_scale = FiniteFloat(
      Required(scales, "angular_velocity", "policy.scales"),
      "policy.scales.angular_velocity");
  config.joint_position_scale = FiniteFloat(
      Required(scales, "joint_position", "policy.scales"),
      "policy.scales.joint_position");
  config.joint_velocity_scale = FiniteFloat(
      Required(scales, "joint_velocity", "policy.scales"),
      "policy.scales.joint_velocity");

  const YAML::Node runtime = RequiredMap(root, "runtime", "root");
  config.motor_slots = NonNegativeSize(
      Required(runtime, "motor_slots", "runtime"), "runtime.motor_slots");
  config.command_dt = FiniteFloat(
      Required(runtime, "command_dt", "runtime"), "runtime.command_dt");

  const YAML::Node robot = RequiredMap(root, "robot", "root");
  config.joint_names = StringArray<kNumJoints>(
      Required(robot, "joint_names", "robot"), "robot.joint_names");
  config.motor_indices = SizeArray<kNumJoints>(
      Required(robot, "motor_indices", "robot"), "robot.motor_indices");
  config.default_q = FloatArray<kNumJoints>(
      Required(robot, "default_positions", "robot"),
      "robot.default_positions");
  config.kp = FloatArray<kNumJoints>(Required(robot, "kp", "robot"),
                                    "robot.kp");
  config.kd = FloatArray<kNumJoints>(Required(robot, "kd", "robot"),
                                    "robot.kd");
  const YAML::Node limits = RequiredMap(robot, "limits", "robot");
  config.hard_lower = FloatArray<kNumJoints>(
      Required(limits, "lower", "robot.limits"), "robot.limits.lower");
  config.hard_upper = FloatArray<kNumJoints>(
      Required(limits, "upper", "robot.limits"), "robot.limits.upper");
  config.velocity_limits = FloatArray<kNumJoints>(
      Required(limits, "velocity", "robot.limits"),
      "robot.limits.velocity");
  config.target_velocity_limits = config.velocity_limits;
  config.effort_limits = FloatArray<kNumJoints>(
      Required(limits, "effort", "robot.limits"), "robot.limits.effort");

  const YAML::Node commands = RequiredMap(root, "commands", "root");
  const auto vx = FloatArray<2>(Required(commands, "vx", "commands"),
                                "commands.vx");
  const auto vy = FloatArray<2>(Required(commands, "vy", "commands"),
                                "commands.vy");
  const auto yaw = FloatArray<2>(Required(commands, "yaw", "commands"),
                                 "commands.yaw");
  config.command_lower = {vx[0], vy[0], yaw[0]};
  config.command_upper = {vx[1], vy[1], yaw[1]};

  const YAML::Node safety = RequiredMap(root, "safety", "root");
  config.low_state_timeout = PositiveMilliseconds(
      Required(safety, "low_state_timeout_ms", "safety"),
      "safety.low_state_timeout_ms");
  config.tick_timeout = config.low_state_timeout;
  config.initial_state_timeout = PositiveMilliseconds(
      Required(safety, "initial_state_timeout_ms", "safety"),
      "safety.initial_state_timeout_ms");
  config.mainboard_prearm_timeout = PositiveMilliseconds(
      Required(safety, "mainboard_prearm_timeout_ms", "safety"),
      "safety.mainboard_prearm_timeout_ms");
  config.mainboard_runtime_timeout = PositiveMilliseconds(
      Required(safety, "mainboard_runtime_timeout_ms", "safety"),
      "safety.mainboard_runtime_timeout_ms");
  config.mode_release_timeout = PositiveMilliseconds(
      Required(safety, "mode_release_timeout_ms", "safety"),
      "safety.mode_release_timeout_ms");
  config.max_tilt_rad = FiniteFloat(
      Required(safety, "max_tilt_rad", "safety"), "safety.max_tilt_rad");
  config.quaternion_norm_min = FiniteFloat(
      Required(safety, "quaternion_norm_min", "safety"),
      "safety.quaternion_norm_min");
  config.quaternion_norm_max = FiniteFloat(
      Required(safety, "quaternion_norm_max", "safety"),
      "safety.quaternion_norm_max");
  config.soft_position_limit_ratio = FiniteFloat(
      Required(safety, "soft_position_limit_ratio", "safety"),
      "safety.soft_position_limit_ratio");
  config.stand_duration = PositiveSeconds(
      Required(safety, "stand_duration_s", "safety"),
      "safety.stand_duration_s");
  config.damping_duration = PositiveSeconds(
      Required(safety, "damping_duration_s", "safety"),
      "safety.damping_duration_s");
  config.stop_duration = PositiveSeconds(
      Required(safety, "stop_duration_s", "safety"),
      "safety.stop_duration_s");

  const YAML::Node sim = RequiredMap(root, "sim", "root");
  config.xml_path = ResolveProjectPath(
      config.source_path,
      Scalar<std::string>(Required(sim, "xml", "sim"), "sim.xml"));
  config.physics_dt = FiniteFloat(
      Required(sim, "physics_dt", "sim"), "sim.physics_dt");
  config.base_height = FiniteFloat(
      Required(sim, "base_height", "sim"), "sim.base_height");
  config.sim_joint_position_tolerance_rad = FiniteFloat(
      Required(sim, "joint_position_tolerance_rad", "sim"),
      "sim.joint_position_tolerance_rad");

  const auto soft_limits = ComputeSoftPositionLimits(
      config.hard_lower, config.hard_upper,
      config.soft_position_limit_ratio);
  config.soft_lower = soft_limits.first;
  config.soft_upper = soft_limits.second;
  config.Validate();
  return config;
}

void A2Config::Validate() const {
  if (config_version != 1) {
    ConfigError("config_version must be 1");
  }
  if (contract_id != kContractId) {
    ConfigError("contract_id must be " + std::string(kContractId));
  }
  if (policy_sha256.size() != 64U ||
      !std::all_of(policy_sha256.begin(), policy_sha256.end(), [](char value) {
        return (value >= '0' && value <= '9') ||
               (value >= 'a' && value <= 'f');
      })) {
    ConfigError("policy.sha256 must be a 64-character lowercase hex digest");
  }
  if (policy_golden_output.has_value() &&
      !AllFinite(*policy_golden_output)) {
    ConfigError("policy.golden_output must contain only finite values");
  }
  if (observation_dim != kObservationDim || action_dim != kActionDim) {
    ConfigError("policy dimensions must be exactly 45 -> 12");
  }
  if (motor_slots != kMotorSlots) {
    ConfigError("runtime.motor_slots must be exactly 35");
  }
  if (!NearlyEqual(policy_dt, kPolicyDt) ||
      !NearlyEqual(command_dt, kCommandDt) ||
      !NearlyEqual(action_scale, kActionScale)) {
    ConfigError("policy/action/command timing does not match the frozen ABI");
  }
  if (!NearlyEqual(angular_velocity_scale, 0.25F) ||
      !NearlyEqual(joint_position_scale, 1.0F) ||
      !NearlyEqual(joint_velocity_scale, 0.05F)) {
    ConfigError("observation scales do not match the frozen A2 ABI");
  }
  RequireArrayEqual(command_scale, kCommandScale,
                    "policy.scales.command");
  RequireArrayEqual(command_lower, kCommandLower, "commands lower bounds");
  RequireArrayEqual(command_upper, kCommandUpper, "commands upper bounds");

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (joint_names[i] != kJointNames[i]) {
      ConfigError("robot.joint_names does not match policy order at index " +
                  std::to_string(i));
    }
    if (motor_indices[i] != i) {
      ConfigError("robot.motor_indices must map policy joints to slots 0..11");
    }
  }
  RequireArrayEqual(default_q, kDefaultPositions, "robot.default_positions");
  RequireArrayEqual(kp, kPositionKp, "robot.kp");
  RequireArrayEqual(kd, kPositionKd, "robot.kd");
  RequireArrayEqual(hard_lower, kHardLower, "robot.limits.lower");
  RequireArrayEqual(hard_upper, kHardUpper, "robot.limits.upper");
  RequireArrayEqual(velocity_limits, kVelocityLimits,
                    "robot.limits.velocity");
  RequireArrayEqual(target_velocity_limits, kVelocityLimits,
                    "target velocity limits");
  RequireArrayEqual(effort_limits, kEffortLimits, "robot.limits.effort");

  if (model_path.empty() || xml_path.empty()) {
    ConfigError("policy.file and sim.xml must not be empty");
  }
  if (!std::isfinite(physics_dt) || physics_dt <= 0.0F ||
      !std::isfinite(base_height) || base_height <= 0.0F ||
      !std::isfinite(sim_joint_position_tolerance_rad) ||
      sim_joint_position_tolerance_rad < 0.0F ||
      sim_joint_position_tolerance_rad > 0.01F) {
    ConfigError(
        "simulation timestep/base height must be positive and joint "
        "position tolerance must be in [0, 0.01] rad");
  }
  const float sim_decimation = policy_dt / physics_dt;
  const float command_decimation = policy_dt / command_dt;
  if (!NearlyEqual(sim_decimation, std::round(sim_decimation)) ||
      !NearlyEqual(command_decimation, std::round(command_decimation))) {
    ConfigError("policy dt must be an integer multiple of command/sim dt");
  }

  if (low_state_timeout.count() <= 0 || tick_timeout.count() <= 0 ||
      initial_state_timeout.count() <= 0 ||
      mainboard_prearm_timeout.count() <= 0 ||
      mainboard_runtime_timeout.count() <= 0 ||
      mode_release_timeout.count() <= 0 || stand_duration.count() <= 0 ||
      damping_duration.count() <= 0 || stop_duration.count() <= 0) {
    ConfigError("all safety durations must be positive");
  }
  if (low_state_timeout != std::chrono::milliseconds(100) ||
      tick_timeout != std::chrono::milliseconds(100) ||
      initial_state_timeout != std::chrono::milliseconds(5000) ||
      mainboard_prearm_timeout != std::chrono::milliseconds(2000) ||
      mainboard_runtime_timeout != std::chrono::milliseconds(2000) ||
      mode_release_timeout != std::chrono::milliseconds(30000) ||
      stand_duration != std::chrono::milliseconds(2000) ||
      damping_duration != std::chrono::milliseconds(500) ||
      stop_duration != std::chrono::milliseconds(100)) {
    ConfigError("safety timing values do not match the frozen A2 contract");
  }
  if (!std::isfinite(max_tilt_rad) || max_tilt_rad <= 0.0F ||
      max_tilt_rad >= 3.14159265358979323846F ||
      !std::isfinite(quaternion_norm_min) ||
      !std::isfinite(quaternion_norm_max) || quaternion_norm_min <= 0.0F ||
      quaternion_norm_min >= quaternion_norm_max ||
      !std::isfinite(soft_position_limit_ratio) ||
      soft_position_limit_ratio <= 0.0F ||
      soft_position_limit_ratio > 1.0F) {
    ConfigError("invalid orientation or soft-limit safety threshold");
  }
  if (!NearlyEqual(max_tilt_rad, 1.0F) ||
      !NearlyEqual(quaternion_norm_min, 0.9F) ||
      !NearlyEqual(quaternion_norm_max, 1.1F) ||
      !NearlyEqual(soft_position_limit_ratio, 0.9F)) {
    ConfigError("safety thresholds do not match the frozen A2 contract");
  }
  if (expected_mode_machine != kExpectedModeMachine ||
      expected_form != kExpectedForm) {
    ConfigError("A2 mode_machine/form contract must be 1/\"0\"");
  }

  const auto expected_soft = ComputeSoftPositionLimits(
      hard_lower, hard_upper, soft_position_limit_ratio);
  RequireArrayEqual(soft_lower, expected_soft.first, "soft lower limits");
  RequireArrayEqual(soft_upper, expected_soft.second, "soft upper limits");
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (default_q[i] < hard_lower[i] || default_q[i] > hard_upper[i]) {
      ConfigError("default joint position lies outside the hard range");
    }
  }
}

}  // namespace a2

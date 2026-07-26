#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace a2 {

inline constexpr std::size_t kNumJoints = 12;
inline constexpr std::size_t kObservationDim = 45;
inline constexpr std::size_t kActionDim = 12;
inline constexpr std::size_t kMotorSlots = 35;

inline constexpr std::string_view kContractId = "a2_45d_project_v0";
inline constexpr std::string_view kBundledPolicySha256 =
    "324d851114f77bb848255026bd56d8d4ebe00a72a72aac54f4271cd644c6fb65";

inline constexpr float kPolicyDt = 0.02F;
inline constexpr float kCommandDt = 0.002F;
inline constexpr float kActionScale = 0.25F;
inline constexpr std::uint8_t kExpectedModeMachine = 1;
inline constexpr std::string_view kExpectedForm = "0";

using Vec3 = std::array<float, 3>;
using JointArray = std::array<float, kNumJoints>;
using Observation = std::array<float, kObservationDim>;

inline constexpr std::array<std::string_view, kNumJoints> kJointNames = {
    "FR_hip_joint",   "FR_thigh_joint", "FR_calf_joint",
    "FL_hip_joint",   "FL_thigh_joint", "FL_calf_joint",
    "RR_hip_joint",   "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint",   "RL_thigh_joint", "RL_calf_joint",
};

inline constexpr JointArray kDefaultPositions = {
    0.1F, 0.9F, -1.8F, -0.1F, 0.9F, -1.8F,
    0.1F, 0.9F, -1.8F, -0.1F, 0.9F, -1.8F,
};

inline constexpr JointArray kPositionKp = {
    100.0F, 100.0F, 150.0F, 100.0F, 100.0F, 150.0F,
    100.0F, 100.0F, 150.0F, 100.0F, 100.0F, 150.0F,
};

inline constexpr JointArray kPositionKd = {
    4.0F, 4.0F, 6.0F, 4.0F, 4.0F, 6.0F,
    4.0F, 4.0F, 6.0F, 4.0F, 4.0F, 6.0F,
};

inline constexpr JointArray kHardLower = {
    -1.01F, -2.34F, -2.77F, -1.01F, -2.34F, -2.77F,
    -1.01F, -1.56F, -2.77F, -1.01F, -1.56F, -2.77F,
};

inline constexpr JointArray kHardUpper = {
    1.01F, 3.15F, -0.54F, 1.01F, 3.15F, -0.54F,
    1.01F, 3.94F, -0.54F, 1.01F, 3.94F, -0.54F,
};

inline constexpr JointArray kVelocityLimits = {
    22.0F, 22.0F, 14.6667F, 22.0F, 22.0F, 14.6667F,
    22.0F, 22.0F, 14.6667F, 22.0F, 22.0F, 14.6667F,
};

inline constexpr JointArray kEffortLimits = {
    120.0F, 120.0F, 180.0F, 120.0F, 120.0F, 180.0F,
    120.0F, 120.0F, 180.0F, 120.0F, 120.0F, 180.0F,
};

inline constexpr Vec3 kCommandLower = {-0.5F, -0.5F, -1.0F};
inline constexpr Vec3 kCommandUpper = {1.0F, 0.5F, 1.0F};
inline constexpr Vec3 kCommandScale = {1.0F, 1.0F, 0.25F};

template <std::size_t N>
inline bool AllFinite(const std::array<float, N>& values) noexcept {
  for (const float value : values) {
    if (!std::isfinite(value)) {
      return false;
    }
  }
  return true;
}

inline std::size_t JointIndex(const std::string_view name) {
  const auto it = std::find(kJointNames.begin(), kJointNames.end(), name);
  if (it == kJointNames.end()) {
    throw std::invalid_argument("unknown A2 policy joint name");
  }
  return static_cast<std::size_t>(std::distance(kJointNames.begin(), it));
}

struct ObservationInput {
  Vec3 command{};
  Vec3 projected_gravity{};
  Vec3 body_angular_velocity{};
  JointArray joint_position{};
  JointArray joint_velocity{};
  JointArray previous_action{};
};

// Policy-visible ABI:
// command(3), projected gravity(3), body angular velocity(3), q-q0(12),
// dq(12), and the previous raw policy action(12).
inline Observation BuildObservation(
    const ObservationInput& input, const JointArray& default_positions,
    const Vec3& command_scale = kCommandScale,
    const float angular_velocity_scale = 0.25F,
    const float joint_position_scale = 1.0F,
    const float joint_velocity_scale = 0.05F) {
  if (!AllFinite(input.command) || !AllFinite(input.projected_gravity) ||
      !AllFinite(input.body_angular_velocity) ||
      !AllFinite(input.joint_position) || !AllFinite(input.joint_velocity) ||
      !AllFinite(input.previous_action) || !AllFinite(default_positions) ||
      !AllFinite(command_scale) || !std::isfinite(angular_velocity_scale) ||
      !std::isfinite(joint_position_scale) ||
      !std::isfinite(joint_velocity_scale)) {
    throw std::invalid_argument("A2 observation contains a non-finite value");
  }

  Observation observation{};
  for (std::size_t i = 0; i < 3; ++i) {
    observation[i] = input.command[i] * command_scale[i];
    observation[3 + i] = input.projected_gravity[i];
    observation[6 + i] =
        input.body_angular_velocity[i] * angular_velocity_scale;
  }
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    observation[9 + i] =
        (input.joint_position[i] - default_positions[i]) *
        joint_position_scale;
    observation[21 + i] = input.joint_velocity[i] * joint_velocity_scale;
    observation[33 + i] = input.previous_action[i];
  }
  return observation;
}

inline Vec3 ClampCommand(const Vec3& command,
                         const Vec3& lower = kCommandLower,
                         const Vec3& upper = kCommandUpper) {
  if (!AllFinite(command) || !AllFinite(lower) || !AllFinite(upper)) {
    throw std::invalid_argument("A2 command bounds contain a non-finite value");
  }
  Vec3 result{};
  for (std::size_t i = 0; i < result.size(); ++i) {
    if (lower[i] > upper[i]) {
      throw std::invalid_argument("A2 command lower bound exceeds upper bound");
    }
    result[i] = std::clamp(command[i], lower[i], upper[i]);
  }
  return result;
}

// Maps already-oriented joystick axes in [-1, 1] to the asymmetric physical
// command envelope while preserving zero. The caller owns controller-specific
// axis selection/signs (for example, LY -> vx and -LX -> vy).
inline Vec3 MapCommandAxes(const Vec3& axes,
                           const Vec3& lower = kCommandLower,
                           const Vec3& upper = kCommandUpper) {
  if (!AllFinite(axes)) {
    throw std::invalid_argument("A2 command axis contains a non-finite value");
  }
  Vec3 command{};
  for (std::size_t i = 0; i < axes.size(); ++i) {
    const float axis = std::clamp(axes[i], -1.0F, 1.0F);
    command[i] = axis >= 0.0F ? axis * upper[i] : (-axis) * lower[i];
  }
  return ClampCommand(command, lower, upper);
}

inline std::pair<JointArray, JointArray> ComputeSoftPositionLimits(
    const JointArray& hard_lower, const JointArray& hard_upper,
    const float ratio) {
  if (!AllFinite(hard_lower) || !AllFinite(hard_upper) ||
      !std::isfinite(ratio) || ratio <= 0.0F || ratio > 1.0F) {
    throw std::invalid_argument("invalid A2 soft position limit parameters");
  }
  JointArray lower{};
  JointArray upper{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (hard_lower[i] >= hard_upper[i]) {
      throw std::invalid_argument("A2 hard position limit interval is empty");
    }
    const float midpoint = 0.5F * (hard_lower[i] + hard_upper[i]);
    const float half_range = 0.5F * ratio * (hard_upper[i] - hard_lower[i]);
    lower[i] = midpoint - half_range;
    upper[i] = midpoint + half_range;
  }
  return {lower, upper};
}

inline JointArray ActionToTarget(const JointArray& raw_action,
                                 const JointArray& default_positions,
                                 const float action_scale = kActionScale) {
  if (!AllFinite(raw_action) || !AllFinite(default_positions) ||
      !std::isfinite(action_scale) || action_scale <= 0.0F) {
    throw std::invalid_argument("invalid A2 policy action");
  }
  JointArray target{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    target[i] = default_positions[i] + action_scale * raw_action[i];
  }
  return target;
}

struct TargetFilterInput {
  JointArray desired_target{};
  JointArray previous_target{};
  JointArray joint_position{};
  JointArray joint_velocity{};
};

struct TargetFilterResult {
  JointArray target{};
  JointArray estimated_effort{};
  std::array<bool, kNumJoints> position_limited{};
  std::array<bool, kNumJoints> velocity_limited{};
  std::array<bool, kNumJoints> effort_limited{};

  bool limited() const noexcept {
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (position_limited[i] || velocity_limited[i] || effort_limited[i]) {
        return true;
      }
    }
    return false;
  }
};

// Final 500 Hz target gate. It applies the soft URDF range, a target slew-rate
// bound, then a conservative PD-only effort bound:
// tau = kp * (q_target - q) - kd * dq.
inline TargetFilterResult FilterTarget(
    const TargetFilterInput& input, const JointArray& soft_lower,
    const JointArray& soft_upper, const JointArray& target_velocity_limits,
    const JointArray& effort_limits, const JointArray& kp,
    const JointArray& kd, const float dt = kCommandDt) {
  if (!AllFinite(input.desired_target) || !AllFinite(input.previous_target) ||
      !AllFinite(input.joint_position) || !AllFinite(input.joint_velocity) ||
      !AllFinite(soft_lower) || !AllFinite(soft_upper) ||
      !AllFinite(target_velocity_limits) || !AllFinite(effort_limits) ||
      !AllFinite(kp) || !AllFinite(kd) || !std::isfinite(dt) || dt <= 0.0F) {
    throw std::invalid_argument("invalid A2 target filter input");
  }

  TargetFilterResult result{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (soft_lower[i] >= soft_upper[i] || target_velocity_limits[i] <= 0.0F ||
        effort_limits[i] <= 0.0F || kp[i] <= 0.0F || kd[i] < 0.0F) {
      throw std::invalid_argument("invalid A2 target filter limit");
    }

    float target =
        std::clamp(input.desired_target[i], soft_lower[i], soft_upper[i]);
    result.position_limited[i] = target != input.desired_target[i];

    const float max_delta = target_velocity_limits[i] * dt;
    const float rate_lower = input.previous_target[i] - max_delta;
    const float rate_upper = input.previous_target[i] + max_delta;
    const float rate_target = std::clamp(target, rate_lower, rate_upper);
    result.velocity_limited[i] = rate_target != target;
    target = rate_target;

    const float effort_target_lower =
        input.joint_position[i] +
        (-effort_limits[i] + kd[i] * input.joint_velocity[i]) / kp[i];
    const float effort_target_upper =
        input.joint_position[i] +
        (effort_limits[i] + kd[i] * input.joint_velocity[i]) / kp[i];
    const float effort_target =
        std::clamp(target, effort_target_lower, effort_target_upper);
    result.effort_limited[i] = effort_target != target;
    result.target[i] = effort_target;
    result.estimated_effort[i] =
        kp[i] * (effort_target - input.joint_position[i]) -
        kd[i] * input.joint_velocity[i];
  }
  return result;
}

}  // namespace a2

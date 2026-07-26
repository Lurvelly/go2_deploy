#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "a2/contract.hpp"

namespace a2 {

struct A2Config {
  std::filesystem::path source_path;
  std::filesystem::path model_path;
  std::filesystem::path xml_path;

  std::string contract_id{std::string(kContractId)};
  std::string policy_sha256{std::string(kBundledPolicySha256)};
  std::optional<JointArray> policy_golden_output;
  std::size_t config_version{1};
  std::size_t observation_dim{kObservationDim};
  std::size_t action_dim{kActionDim};
  std::size_t motor_slots{kMotorSlots};

  float policy_dt{kPolicyDt};
  float command_dt{kCommandDt};
  float physics_dt{0.005F};
  float action_scale{kActionScale};
  float angular_velocity_scale{0.25F};
  float joint_position_scale{1.0F};
  float joint_velocity_scale{0.05F};
  float base_height{0.42F};
  // MuJoCo constraints can overshoot a joint range by a few milliradians.
  // This tolerance is consumed only when RuntimeOptions::simulation is true.
  float sim_joint_position_tolerance_rad{0.005F};

  Vec3 command_lower{kCommandLower};
  Vec3 command_upper{kCommandUpper};
  Vec3 command_scale{kCommandScale};

  std::array<std::string, kNumJoints> joint_names{};
  std::array<std::size_t, kNumJoints> motor_indices{};
  JointArray default_q{kDefaultPositions};
  JointArray kp{kPositionKp};
  JointArray kd{kPositionKd};
  JointArray hard_lower{kHardLower};
  JointArray hard_upper{kHardUpper};
  JointArray soft_lower{};
  JointArray soft_upper{};
  JointArray velocity_limits{kVelocityLimits};
  JointArray target_velocity_limits{kVelocityLimits};
  JointArray effort_limits{kEffortLimits};

  std::chrono::milliseconds low_state_timeout{100};
  std::chrono::milliseconds tick_timeout{100};
  std::chrono::milliseconds initial_state_timeout{5000};
  std::chrono::milliseconds mainboard_prearm_timeout{2000};
  std::chrono::milliseconds mainboard_runtime_timeout{2000};
  std::chrono::milliseconds mode_release_timeout{30000};
  std::chrono::milliseconds stand_duration{2000};
  std::chrono::milliseconds damping_duration{500};
  std::chrono::milliseconds stop_duration{100};

  float max_tilt_rad{1.0F};
  float quaternion_norm_min{0.9F};
  float quaternion_norm_max{1.1F};
  float soft_position_limit_ratio{0.9F};
  std::uint8_t expected_mode_machine{kExpectedModeMachine};
  std::string expected_form{std::string(kExpectedForm)};

  A2Config();

  static A2Config Load(const std::filesystem::path& filename);
  static A2Config LoadFromFile(const std::filesystem::path& filename) {
    return Load(filename);
  }

  // Throws std::runtime_error when any policy ABI, physical limit, or safety
  // invariant is invalid. Load() always calls Validate().
  void Validate() const;
};

}  // namespace a2

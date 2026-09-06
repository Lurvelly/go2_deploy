#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "a2/config.hpp"
#include "a2/contract.hpp"
#include "a2/crc32.hpp"

namespace {

int failures = 0;

void Check(const bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

bool Near(const float lhs, const float rhs, const float tolerance = 1.0e-6F) {
  return std::fabs(lhs - rhs) <= tolerance;
}

void ExpectThrow(const std::function<void()>& callback,
                 const std::string& message) {
  try {
    callback();
    Check(false, message + " (no exception)");
  } catch (const std::exception&) {
  }
}

std::filesystem::path RepositoryRoot() {
  std::filesystem::path source(__FILE__);
  if (source.is_relative()) {
    source = std::filesystem::current_path() / source;
  }
  return source.parent_path().parent_path().lexically_normal();
}

void TestFrozenConstants() {
  Check(a2::kObservationDim == 45, "observation dimension is 45");
  Check(a2::kActionDim == 12, "action dimension is 12");
  Check(a2::kMotorSlots == 35, "HG command has 35 motor slots");
  Check(a2::kContractId == "a2_45d_project_v0", "contract id is pinned");
  Check(a2::kBundledPolicySha256 ==
            "886a653beb628ec09b3287b0e0db79279756e535037679f06d05c29d243b6cc8",
        "bundled policy SHA-256 is pinned");
  Check(a2::kJointNames.front() == "FR_hip_joint",
        "policy order begins with FR");
  Check(a2::kJointNames[3] == "FL_hip_joint", "FL block is second");
  Check(a2::kJointNames[6] == "RR_hip_joint", "RR block is third");
  Check(a2::kJointNames[9] == "RL_hip_joint", "RL block is fourth");
  Check(a2::JointIndex("RR_calf_joint") == 8,
        "joint name lookup preserves policy order");
  ExpectThrow([] { (void)a2::JointIndex("not_an_a2_joint"); },
              "unknown joints are rejected");
}

void TestObservationLayout() {
  a2::ObservationInput input;
  input.command = {1.0F, 2.0F, 3.0F};
  input.projected_gravity = {4.0F, 5.0F, 6.0F};
  input.body_angular_velocity = {8.0F, 12.0F, 16.0F};
  for (std::size_t i = 0; i < a2::kNumJoints; ++i) {
    input.joint_position[i] = a2::kDefaultPositions[i] +
                              static_cast<float>(i + 1);
    input.joint_velocity[i] = 20.0F + static_cast<float>(i);
    input.previous_action[i] = 40.0F + static_cast<float>(i);
  }

  const auto observation = a2::BuildObservation(input, a2::kDefaultPositions);
  Check(Near(observation[0], 1.0F) && Near(observation[1], 2.0F) &&
            Near(observation[2], 0.75F),
        "command occupies [0,3) with yaw scaling");
  Check(Near(observation[3], 4.0F) && Near(observation[5], 6.0F),
        "gravity occupies [3,6)");
  Check(Near(observation[6], 2.0F) && Near(observation[8], 4.0F),
        "angular velocity occupies [6,9) and is scaled");
  Check(Near(observation[9], 1.0F) && Near(observation[20], 12.0F),
        "relative joint position occupies [9,21)");
  Check(Near(observation[21], 1.0F) && Near(observation[32], 1.55F),
        "joint velocity occupies [21,33) and is scaled");
  Check(Near(observation[33], 40.0F) && Near(observation[44], 51.0F),
        "previous raw action occupies [33,45)");

  input.joint_velocity[5] = std::numeric_limits<float>::quiet_NaN();
  ExpectThrow(
      [&] { (void)a2::BuildObservation(input, a2::kDefaultPositions); },
      "non-finite observations fail closed");
}

void TestCommandsActionsAndFiltering() {
  const auto mapped = a2::MapCommandAxes({-1.0F, 0.5F, 2.0F});
  Check(Near(mapped[0], -0.5F) && Near(mapped[1], 0.25F) &&
            Near(mapped[2], 1.0F),
        "joystick mapping preserves zero and asymmetric envelopes");
  const auto clamped = a2::ClampCommand({2.0F, -2.0F, 0.25F});
  Check(Near(clamped[0], 1.0F) && Near(clamped[1], -0.5F) &&
            Near(clamped[2], 0.25F),
        "physical commands are clamped");

  a2::JointArray action{};
  action.fill(2.0F);
  const auto target = a2::ActionToTarget(action, a2::kDefaultPositions);
  Check(Near(target[0], 0.6F) && Near(target[2], -1.3F),
        "q_target = q0 + 0.25 * raw_action");

  const auto soft = a2::ComputeSoftPositionLimits(
      a2::kHardLower, a2::kHardUpper, 0.9F);
  Check(Near(soft.first[0], -0.909F) && Near(soft.second[0], 0.909F),
        "soft limits retain the centered 90 percent interval");

  a2::TargetFilterInput filter_input;
  filter_input.desired_target = target;
  filter_input.previous_target = a2::kDefaultPositions;
  filter_input.joint_position = a2::kDefaultPositions;
  filter_input.joint_velocity.fill(0.0F);
  filter_input.desired_target[0] = 10.0F;
  const auto filtered = a2::FilterTarget(
      filter_input, soft.first, soft.second, a2::kVelocityLimits,
      a2::kEffortLimits, a2::kPositionKp, a2::kPositionKd);
  Check(filtered.position_limited[0] && filtered.velocity_limited[0],
        "target filter reports position and slew limiting");
  Check(filtered.target[0] <= soft.second[0],
        "filtered target remains inside the soft position range");
  for (std::size_t i = 0; i < a2::kNumJoints; ++i) {
    Check(std::fabs(filtered.estimated_effort[i]) <=
              a2::kEffortLimits[i] + 1.0e-4F,
          "estimated PD effort remains bounded");
  }

  // A policy request is held for ten 500 Hz command frames. The limiter must
  // retain that immutable request while only its previous-sent state advances;
  // otherwise a 22 rad/s slew limit degenerates to 2.2 rad/s at 50 Hz.
  a2::TargetFilterInput repeated;
  repeated.desired_target = a2::kDefaultPositions;
  repeated.desired_target[0] += 0.5F;
  repeated.previous_target = a2::kDefaultPositions;
  repeated.joint_position = a2::kDefaultPositions;
  repeated.joint_velocity.fill(0.0F);
  const float expected_step = a2::kVelocityLimits[0] * a2::kCommandDt;
  float previous = repeated.previous_target[0];
  for (std::size_t tick = 1; tick <= 5; ++tick) {
    const auto frame = a2::FilterTarget(
        repeated, soft.first, soft.second, a2::kVelocityLimits,
        a2::kEffortLimits, a2::kPositionKp, a2::kPositionKd);
    Check(Near(frame.target[0],
               a2::kDefaultPositions[0] + expected_step * tick),
          "fixed policy target advances on every 500 Hz command tick");
    Check(frame.target[0] > previous,
          "fixed policy target does not freeze after the first slew step");
    previous = frame.target[0];
    repeated.previous_target = frame.target;
  }
  Check(Near(repeated.desired_target[0],
             a2::kDefaultPositions[0] + 0.5F),
        "500 Hz limiter state never overwrites the policy request");
}

void TestOfficialCrc() {
  const std::array<std::uint32_t, 2> words{0x12345678U, 0x9ABCDEF0U};
  Check(a2::Crc32Words(words.data(), words.size()) == 0x7D24A31BU,
        "CRC matches the official MSB-first word algorithm");
  Check(a2::Crc32Words(words.data(), 1) == 0xDF8A8A2BU,
        "CRC one-word reference vector matches");

  struct Message {
    std::uint32_t first;
    std::uint32_t second;
    std::uint32_t crc;
  } message{words[0], words[1], 0U};
  message.crc = a2::ComputeMessageCrc(message);
  Check(message.crc == 0x7D24A31BU && a2::HasValidTrailingCrc(message),
        "trailing DDS CRC helper accepts an intact message");
  message.second ^= 1U;
  Check(!a2::HasValidTrailingCrc(message),
        "trailing DDS CRC helper rejects mutation");
}

void TestConfiguration() {
  const auto config_path = RepositoryRoot() / "params" / "a2.yaml";
  const a2::A2Config config = a2::A2Config::Load(config_path);
  Check(config.source_path == std::filesystem::absolute(config_path),
        "configuration records its absolute source path");
  Check(config.model_path == RepositoryRoot() / "models" /
                                 "a2_45d_policy_0804.jit",
        "model path is resolved relative to the project root");
  Check(config.xml_path == RepositoryRoot() / "assets" / "a2" / "scene.xml",
        "MJCF path is resolved relative to the project root");
  Check(config.observation_dim == 45 && config.action_dim == 12,
        "configuration keeps the 45 -> 12 ABI");
  Check(config.policy_golden_output.has_value(),
        "bundled policy config pins a reference output");
  Check(config.motor_indices[11] == 11,
        "configuration maps the first twelve HG motor slots");
  Check(config.low_state_timeout == std::chrono::milliseconds(100),
        "configuration loads the 100 ms watchdog");
  Check(config.stand_duration == std::chrono::milliseconds(2000),
        "configuration converts stand seconds to milliseconds");
  Check(Near(config.sim_joint_position_tolerance_rad, 0.005F),
        "configuration loads the bounded 5 mrad MuJoCo tolerance");

  a2::A2Config invalid = config;
  invalid.observation_dim = 47;
  ExpectThrow([&] { invalid.Validate(); },
              "non-45D policy configuration is rejected");
  invalid = config;
  invalid.policy_sha256 = "not-a-sha256";
  ExpectThrow([&] { invalid.Validate(); },
              "malformed policy hashes are rejected");
  invalid = config;
  invalid.policy_sha256.assign(64, 'a');
  try {
    invalid.Validate();
  } catch (const std::exception&) {
    Check(false, "a different well-formed policy identity is accepted");
  }
  invalid = config;
  invalid.joint_names[0] = "FL_hip_joint";
  ExpectThrow([&] { invalid.Validate(); },
              "joint-order drift is rejected");
  invalid = config;
  invalid.low_state_timeout = std::chrono::milliseconds(1000);
  ExpectThrow([&] { invalid.Validate(); },
              "weakening the LowState watchdog is rejected");
  invalid = config;
  invalid.max_tilt_rad = 2.0F;
  ExpectThrow([&] { invalid.Validate(); },
              "weakening the body-tilt gate is rejected");
  invalid = config;
  invalid.sim_joint_position_tolerance_rad = 0.02F;
  ExpectThrow([&] { invalid.Validate(); },
              "excessive MuJoCo joint tolerance is rejected");

  std::ifstream input(config_path);
  std::ostringstream contents;
  contents << input.rdbuf();
  std::string malformed = contents.str();
  const std::string valid_indices =
      "motor_indices: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]";
  const auto position = malformed.find(valid_indices);
  Check(position != std::string::npos,
        "canonical fixture contains the motor-index array");
  if (position != std::string::npos) {
    malformed.replace(position, valid_indices.size(),
                      "motor_indices: [0, 1]");
    const auto malformed_path =
        std::filesystem::temp_directory_path() /
        "a2_deploy_malformed_array_test.yaml";
    {
      std::ofstream output(malformed_path);
      output << malformed;
    }
    ExpectThrow([&] { (void)a2::A2Config::Load(malformed_path); },
                "wrong-length YAML arrays are rejected");
    std::error_code remove_error;
    std::filesystem::remove(malformed_path, remove_error);
  }
}

}  // namespace

int main() {
  TestFrozenConstants();
  TestObservationLayout();
  TestCommandsActionsAndFiltering();
  TestOfficialCrc();
  TestConfiguration();
  if (failures != 0) {
    std::cerr << failures << " contract test(s) failed\n";
    return 1;
  }
  std::cout << "A2 contract tests passed\n";
  return 0;
}

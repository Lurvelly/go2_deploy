#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

#include "a2/contract.hpp"
#include "a2/crc32.hpp"
#include "a2/safety.hpp"

#include <unitree/idl/hg/LowCmd_.hpp>

namespace a2 {

enum class LowCommandKind {
  kStop = 0,
  kDamping = 1,
  kPosition = 2,
};

struct LowCommandBuildInput {
  LowCommandKind kind{LowCommandKind::kStop};
  std::uint8_t mode_machine{0};
  std::array<std::size_t, kNumJoints> motor_indices{};
  JointArray joint_position{};
  JointArray joint_velocity{};
  JointArray target_position{};
  JointArray kp{};
  JointArray kd{};
  JointArray effort_limits{};
};

// This is intentionally a pure, fresh-message builder. Every call starts from
// a value-initialized 35-slot HG command, so a FOC -> STOP transition cannot
// retain motor modes or gains from the preceding frame.
inline unitree_hg::msg::dds_::LowCmd_ BuildLowCommand(
    const LowCommandBuildInput& input) {
  unitree_hg::msg::dds_::LowCmd_ command{};
  command.mode_pr() = 0;
  command.mode_machine() = input.mode_machine;

  if (input.kind != LowCommandKind::kStop) {
    if (!AllFinite(input.joint_position) ||
        !AllFinite(input.joint_velocity) || !AllFinite(input.kd)) {
      throw std::invalid_argument("non-finite A2 low-command input");
    }
    if (input.kind == LowCommandKind::kPosition &&
        (!AllFinite(input.target_position) || !AllFinite(input.kp))) {
      throw std::invalid_argument("non-finite A2 position command");
    }
    if (input.kind == LowCommandKind::kDamping &&
        !AllFinite(input.effort_limits)) {
      throw std::invalid_argument("non-finite A2 damping limit");
    }

    std::array<bool, kMotorSlots> assigned{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const std::size_t motor = input.motor_indices[i];
      if (motor >= command.motor_cmd().size() || assigned[motor]) {
        throw std::out_of_range(
            "A2 motor mapping must contain 12 unique HG slots");
      }
      assigned[motor] = true;

      auto& motor_command = command.motor_cmd()[motor];
      motor_command.mode() = 0x01;
      motor_command.dq() = 0.0F;
      motor_command.tau() = 0.0F;
      if (input.kind == LowCommandKind::kDamping) {
        if (input.kd[i] < 0.0F || input.effort_limits[i] <= 0.0F) {
          throw std::invalid_argument("invalid A2 damping gain or limit");
        }
        motor_command.q() = input.joint_position[i];
        motor_command.kp() = 0.0F;
        const float velocity = std::abs(input.joint_velocity[i]);
        motor_command.kd() =
            velocity > 1.0e-6F
                ? std::min(input.kd[i], input.effort_limits[i] / velocity)
                : input.kd[i];
      } else {
        if (input.kp[i] < 0.0F || input.kd[i] < 0.0F) {
          throw std::invalid_argument("invalid A2 position gain");
        }
        motor_command.q() = input.target_position[i];
        motor_command.kp() = input.kp[i];
        motor_command.kd() = input.kd[i];
      }
    }
  }

  command.crc() = ComputeMessageCrc(command);
  return command;
}

inline bool FaultDampingAuthorized(const SafetyDecision& decision,
                                   const bool state_is_fresh,
                                   const bool foc_was_published) noexcept {
  return state_is_fresh && foc_was_published && decision.foc_allowed &&
         decision.command_authority == CommandAuthority::kDampingOnly;
}

inline bool ShutdownDampingAuthorized(const SafetyDecision& decision,
                                      const bool state_is_fresh,
                                      const bool foc_was_published) noexcept {
  return state_is_fresh && foc_was_published && decision.foc_allowed &&
         (decision.command_authority == CommandAuthority::kActive ||
          decision.command_authority == CommandAuthority::kDampingOnly);
}

}  // namespace a2

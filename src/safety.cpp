#include "a2/safety.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace a2 {
namespace {

template <typename Duration>
std::chrono::milliseconds AgeMilliseconds(
    const SafetySupervisor::TimePoint now,
    const SafetySupervisor::TimePoint then, const Duration maximum) {
  if (now <= then) {
    return std::chrono::milliseconds(0);
  }
  const auto age =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - then);
  return std::min(age, std::chrono::duration_cast<std::chrono::milliseconds>(
                           maximum));
}

bool IsPositiveSerialDelta(const std::uint32_t newer,
                           const std::uint32_t older) noexcept {
  return static_cast<std::int32_t>(newer - older) > 0;
}

bool FaultAllowsDamping(const FaultCode fault) noexcept {
  // State, mainboard, and motion-authority faults must revoke FOC
  // immediately. Bounded damping is reserved for an external/controller
  // failure while the complete LowState trust gate is still healthy.
  return fault == FaultCode::kExternalFault;
}

}  // namespace

const char* FaultCodeName(const FaultCode fault) noexcept {
  switch (fault) {
    case FaultCode::kNone:
      return "none";
    case FaultCode::kInvalidLowStateCrc:
      return "invalid_low_state_crc";
    case FaultCode::kInitialLowStateTimeout:
      return "initial_low_state_timeout";
    case FaultCode::kLowStateTimeout:
      return "low_state_timeout";
    case FaultCode::kTickNotAdvancing:
      return "tick_not_advancing";
    case FaultCode::kUnexpectedModeMachine:
      return "unexpected_mode_machine";
    case FaultCode::kNonFiniteState:
      return "non_finite_state";
    case FaultCode::kQuaternionNorm:
      return "quaternion_norm";
    case FaultCode::kExcessiveTilt:
      return "excessive_tilt";
    case FaultCode::kJointPositionLimit:
      return "joint_position_limit";
    case FaultCode::kJointVelocityLimit:
      return "joint_velocity_limit";
    case FaultCode::kMainboardFault:
      return "mainboard_fault";
    case FaultCode::kMainboardPrearmTimeout:
      return "mainboard_prearm_timeout";
    case FaultCode::kMainboardRuntimeTimeout:
      return "mainboard_runtime_timeout";
    case FaultCode::kMotionCheckFailed:
      return "motion_check_failed";
    case FaultCode::kUnexpectedForm:
      return "unexpected_form";
    case FaultCode::kMotionModeNotReleased:
      return "motion_mode_not_released";
    case FaultCode::kMotionReleaseTimeout:
      return "motion_release_timeout";
    case FaultCode::kExternalFault:
      return "external_fault";
  }
  return "unknown";
}

SafetySupervisor::SafetySupervisor(const A2Config& config,
                                   const TimePoint start_time,
                                   const float joint_position_tolerance_rad)
    : config_(config),
      joint_position_tolerance_rad_(joint_position_tolerance_rad),
      start_time_(start_time),
      prearm_start_time_(start_time) {
  if (!std::isfinite(joint_position_tolerance_rad_) ||
      joint_position_tolerance_rad_ < 0.0F ||
      joint_position_tolerance_rad_ > 0.01F) {
    throw std::invalid_argument(
        "joint position tolerance must be finite and in [0, 0.01] rad");
  }
}

float SafetySupervisor::QuaternionTiltRad(
    const std::array<float, 4>& quaternion) noexcept {
  if (!AllFinite(quaternion)) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  const float norm_squared =
      quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
      quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3];
  if (!std::isfinite(norm_squared) || norm_squared <= 0.0F) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  // World-up dot body-up for q=[w,x,y,z]. This rejects roll/pitch while
  // remaining invariant to yaw.
  const float up_dot = std::clamp(
      1.0F - 2.0F *
                 (quaternion[1] * quaternion[1] +
                  quaternion[2] * quaternion[2]) /
                 norm_squared,
      -1.0F, 1.0F);
  return std::acos(up_dot);
}

void SafetySupervisor::ObserveLowState(const LowStateSnapshot& state,
                                       const TimePoint now) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_low_state_safe_ = false;

  if (!state.crc_valid) {
    LatchFaultLocked(FaultCode::kInvalidLowStateCrc,
                     "LowState CRC32 validation failed", now);
    return;
  }
  if (state.mode_machine != config_.expected_mode_machine) {
    std::ostringstream reason;
    reason << "LowState mode_machine="
           << static_cast<unsigned int>(state.mode_machine)
           << ", expected "
           << static_cast<unsigned int>(config_.expected_mode_machine);
    LatchFaultLocked(FaultCode::kUnexpectedModeMachine, reason.str(), now);
    return;
  }
  if (!AllFinite(state.quaternion) ||
      !AllFinite(state.body_angular_velocity) ||
      !AllFinite(state.joint_position) || !AllFinite(state.joint_velocity)) {
    LatchFaultLocked(FaultCode::kNonFiniteState,
                     "LowState contains a non-finite value", now);
    return;
  }

  float norm_squared = 0.0F;
  for (const float component : state.quaternion) {
    norm_squared += component * component;
  }
  const float norm = std::sqrt(norm_squared);
  if (!std::isfinite(norm) || norm < config_.quaternion_norm_min ||
      norm > config_.quaternion_norm_max) {
    std::ostringstream reason;
    reason << "IMU quaternion norm=" << norm << " is outside ["
           << config_.quaternion_norm_min << ", "
           << config_.quaternion_norm_max << "]";
    LatchFaultLocked(FaultCode::kQuaternionNorm, reason.str(), now);
    return;
  }
  const float tilt = QuaternionTiltRad(state.quaternion);
  if (!std::isfinite(tilt) || tilt > config_.max_tilt_rad) {
    std::ostringstream reason;
    reason << "body tilt=" << tilt << " rad exceeds "
           << config_.max_tilt_rad << " rad";
    LatchFaultLocked(FaultCode::kExcessiveTilt, reason.str(), now);
    return;
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (state.joint_position[i] <
            config_.hard_lower[i] - joint_position_tolerance_rad_ ||
        state.joint_position[i] >
            config_.hard_upper[i] + joint_position_tolerance_rad_) {
      std::ostringstream reason;
      reason << config_.joint_names[i] << " position="
             << state.joint_position[i] << " is outside ["
             << config_.hard_lower[i] << ", " << config_.hard_upper[i] << "]";
      if (joint_position_tolerance_rad_ > 0.0F) {
        reason << " plus simulation tolerance +/-"
               << joint_position_tolerance_rad_ << " rad";
      }
      LatchFaultLocked(FaultCode::kJointPositionLimit, reason.str(), now);
      return;
    }
    if (std::fabs(state.joint_velocity[i]) > config_.velocity_limits[i]) {
      std::ostringstream reason;
      reason << config_.joint_names[i] << " velocity="
             << state.joint_velocity[i] << " exceeds +/-"
             << config_.velocity_limits[i];
      LatchFaultLocked(FaultCode::kJointVelocityLimit, reason.str(), now);
      return;
    }
  }

  latest_low_state_safe_ = true;
  have_low_state_receipt_ = true;
  last_low_state_receipt_time_ = now;
  if (!have_tick_ || IsPositiveSerialDelta(state.tick, last_tick_)) {
    have_tick_ = true;
    last_tick_ = state.tick;
    last_tick_progress_time_ = now;
    last_valid_low_state_time_ = now;
    have_valid_low_state_ = true;
  }
}

void SafetySupervisor::ObserveMainboard(const std::uint32_t state0,
                                        const TimePoint now) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++mainboard_generation_;
  have_mainboard_ = true;
  last_mainboard_time_ = now;
  mainboard_state0_ = state0;
  latest_mainboard_safe_ = state0 == 0U;
  if (arm_requested_ && mainboard_generation_ > arm_mainboard_generation_) {
    if (latest_mainboard_safe_) {
      mainboard_arm_gate_satisfied_ = true;
    } else if (!ever_foc_allowed_ && state0 == 1U) {
      // A pending bit-0 sample supersedes an earlier zero until another zero
      // is observed; queued mode requests cannot bypass this gate.
      mainboard_arm_gate_satisfied_ = false;
    }
  }
  // During the very first pre-arm only bit 0 is allowed to be pending while
  // the A2 mainboard clears it. All other bits are immediate faults, and once
  // FOC has ever been enabled even bit 0 is a runtime fault.
  const bool pending_prearm_bit0 = !ever_foc_allowed_ && state0 == 1U;
  if (!latest_mainboard_safe_ && !pending_prearm_bit0) {
    std::ostringstream reason;
    reason << "mainboard state[0] fault bits set: 0x" << std::hex << state0;
    LatchFaultLocked(FaultCode::kMainboardFault, reason.str(), now);
  }
}

void SafetySupervisor::ObserveMotionStatus(
    const bool check_succeeded, const std::string& form,
    const std::string& mode, const bool released, const TimePoint now) {
  std::lock_guard<std::mutex> lock(mutex_);
  have_motion_status_ = true;
  last_motion_status_time_ = now;
  motion_check_succeeded_ = check_succeeded;
  motion_form_ = form;
  motion_mode_ = mode;
  motion_released_ = released;

  if (!check_succeeded) {
    LatchFaultLocked(FaultCode::kMotionCheckFailed,
                     "MotionSwitcher CheckMode failed", now);
    return;
  }
  if (form != config_.expected_form) {
    LatchFaultLocked(FaultCode::kUnexpectedForm,
                     "MotionSwitcher form=\"" + form + "\", expected \"" +
                         config_.expected_form + "\"",
                     now);
    return;
  }
  if (!released || !mode.empty()) {
    LatchFaultLocked(FaultCode::kMotionModeNotReleased,
                     mode.empty()
                         ? "MotionSwitcher did not confirm released mode"
                         : "MotionSwitcher mode is active: " + mode,
                     now);
  }
}

bool SafetySupervisor::RequestArm(const TimePoint now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fault_ != FaultCode::kNone) {
    return false;
  }
  if (!arm_requested_) {
    prearm_start_time_ = now;
    arm_mainboard_generation_ = mainboard_generation_;
    mainboard_arm_gate_satisfied_ = false;
  }
  arm_requested_ = true;
  control_mode_ = ControlMode::kDamping;
  return true;
}

void SafetySupervisor::Disarm() {
  std::lock_guard<std::mutex> lock(mutex_);
  arm_requested_ = false;
  mainboard_arm_gate_satisfied_ = false;
  control_mode_ = ControlMode::kDamping;
}

bool SafetySupervisor::RequestMode(const ControlMode requested) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!arm_requested_ || fault_ != FaultCode::kNone) {
    return false;
  }
  if (requested == ControlMode::kDamping) {
    control_mode_ = requested;
    return true;
  }
  if (!mainboard_arm_gate_satisfied_ || !ever_foc_allowed_) {
    return false;
  }
  if (requested == ControlMode::kStand &&
      control_mode_ == ControlMode::kDamping) {
    control_mode_ = requested;
    return true;
  }
  if (requested == ControlMode::kControl &&
      control_mode_ == ControlMode::kStand) {
    control_mode_ = requested;
    return true;
  }
  return false;
}

bool SafetySupervisor::ExplicitRearm(const TimePoint now) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (fault_ == FaultCode::kNone || !InputsRecoveredLocked(now)) {
    return false;
  }
  if (now < fault_time_ + config_.damping_duration + config_.stop_duration) {
    return false;
  }

  fault_ = FaultCode::kNone;
  fault_reason_.clear();
  arm_requested_ = true;
  prearm_start_time_ = now;
  arm_mainboard_generation_ = mainboard_generation_;
  mainboard_arm_gate_satisfied_ = false;
  control_mode_ = ControlMode::kDamping;
  return true;
}

void SafetySupervisor::ForceFault(const FaultCode fault,
                                  const std::string& reason,
                                  const TimePoint now) {
  std::lock_guard<std::mutex> lock(mutex_);
  LatchFaultLocked(fault == FaultCode::kNone ? FaultCode::kExternalFault
                                             : fault,
                   reason.empty() ? "external safety fault" : reason, now);
}

SafetyDecision SafetySupervisor::Evaluate(const TimePoint now) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (fault_ == FaultCode::kNone) {
    if (!have_valid_low_state_) {
      if (now >= start_time_ + config_.initial_state_timeout) {
        LatchFaultLocked(FaultCode::kInitialLowStateTimeout,
                         "no valid LowState arrived before startup timeout",
                         now);
      }
    } else if (now > last_valid_low_state_time_ + config_.low_state_timeout) {
      const bool duplicate_tick_stream =
          have_low_state_receipt_ &&
          now <= last_low_state_receipt_time_ + config_.low_state_timeout;
      if (duplicate_tick_stream &&
          now > last_tick_progress_time_ + config_.tick_timeout) {
        LatchFaultLocked(FaultCode::kTickNotAdvancing,
                         "LowState tick stopped advancing", now);
      } else {
        LatchFaultLocked(FaultCode::kLowStateTimeout,
                         "valid LowState stream timed out", now);
      }
    }

    if (fault_ == FaultCode::kNone && arm_requested_) {
      if (!mainboard_arm_gate_satisfied_) {
        if (now >= prearm_start_time_ + config_.mainboard_prearm_timeout) {
          LatchFaultLocked(
              FaultCode::kMainboardPrearmTimeout,
              "mainboard did not publish a fresh zero state during pre-arm",
              now);
        }
      } else if (ever_foc_allowed_ &&
                 now > last_mainboard_time_ +
                           config_.mainboard_runtime_timeout) {
        LatchFaultLocked(FaultCode::kMainboardRuntimeTimeout,
                         "mainboard state stream timed out", now);
      } else if (!ever_foc_allowed_ &&
                 !CurrentMainboardHealthyLocked(now) &&
                 now >= prearm_start_time_ +
                            config_.mainboard_prearm_timeout) {
        LatchFaultLocked(
            FaultCode::kMainboardPrearmTimeout,
            "post-arm zero mainboard state became stale before FOC", now);
      }
    }

    if (fault_ == FaultCode::kNone && arm_requested_ &&
        !MotionHealthyLocked() &&
        now >= prearm_start_time_ + config_.mode_release_timeout) {
      if (have_motion_status_ && !motion_check_succeeded_) {
        LatchFaultLocked(FaultCode::kMotionCheckFailed,
                         "MotionSwitcher CheckMode did not recover", now);
      } else if (have_motion_status_ && motion_form_ != config_.expected_form) {
        LatchFaultLocked(FaultCode::kUnexpectedForm,
                         "MotionSwitcher reported an unexpected form", now);
      } else if (have_motion_status_ && !motion_mode_.empty()) {
        LatchFaultLocked(FaultCode::kMotionModeNotReleased,
                         "MotionSwitcher mode is still active: " +
                             motion_mode_,
                         now);
      } else {
        LatchFaultLocked(FaultCode::kMotionReleaseTimeout,
                         "motion release was not confirmed before timeout",
                         now);
      }
    }
  }

  SafetyDecision decision = BuildDecisionLocked(now);
  if (decision.command_authority == CommandAuthority::kActive) {
    ever_foc_allowed_ = true;
  }
  return decision;
}

bool SafetySupervisor::fault_latched() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return fault_ != FaultCode::kNone;
}

bool SafetySupervisor::armed() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return arm_requested_;
}

ControlMode SafetySupervisor::control_mode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return control_mode_;
}

bool SafetySupervisor::LowStateHealthyLocked(const TimePoint now) const {
  return have_valid_low_state_ && latest_low_state_safe_ &&
         now <= last_valid_low_state_time_ + config_.low_state_timeout &&
         now <= last_tick_progress_time_ + config_.tick_timeout;
}

bool SafetySupervisor::CurrentMainboardHealthyLocked(
    const TimePoint now) const {
  return have_mainboard_ && latest_mainboard_safe_ &&
         mainboard_state0_ == 0U &&
         now <= last_mainboard_time_ + config_.mainboard_runtime_timeout;
}

bool SafetySupervisor::MainboardHealthyLocked(const TimePoint now) const {
  return CurrentMainboardHealthyLocked(now) &&
         (!arm_requested_ || mainboard_arm_gate_satisfied_);
}

bool SafetySupervisor::MotionHealthyLocked() const {
  return have_motion_status_ && motion_check_succeeded_ &&
         motion_form_ == config_.expected_form && motion_mode_.empty() &&
         motion_released_;
}

bool SafetySupervisor::InputsHealthyLocked(const TimePoint now) const {
  return LowStateHealthyLocked(now) && MainboardHealthyLocked(now) &&
         MotionHealthyLocked();
}

bool SafetySupervisor::InputsRecoveredLocked(const TimePoint now) const {
  return LowStateHealthyLocked(now) && CurrentMainboardHealthyLocked(now) &&
         MotionHealthyLocked();
}

void SafetySupervisor::LatchFaultLocked(const FaultCode fault,
                                        const std::string& reason,
                                        const TimePoint now) {
  if (fault_ != FaultCode::kNone) {
    return;
  }
  fault_ = fault == FaultCode::kNone ? FaultCode::kExternalFault : fault;
  fault_reason_ = reason.empty() ? FaultCodeName(fault_) : reason;
  fault_time_ = now;
  control_mode_ = ControlMode::kDamping;
}

SafetyDecision SafetySupervisor::BuildDecisionLocked(
    const TimePoint now) const {
  SafetyDecision decision;
  decision.armed = arm_requested_;
  decision.fault_latched = fault_ != FaultCode::kNone;
  decision.fault = fault_;
  decision.enforced_mode =
      decision.fault_latched ? ControlMode::kDamping : control_mode_;

  if (have_valid_low_state_) {
    decision.low_state_age = AgeMilliseconds(
        now, last_valid_low_state_time_, std::chrono::hours(24));
  }
  if (have_mainboard_) {
    decision.mainboard_age = AgeMilliseconds(
        now, last_mainboard_time_, std::chrono::hours(24));
  }

  if (decision.fault_latched) {
    decision.reason = fault_reason_;
    if (ever_foc_allowed_ && FaultAllowsDamping(fault_) &&
        LowStateHealthyLocked(now) && CurrentMainboardHealthyLocked(now) &&
        MotionHealthyLocked() &&
        now < fault_time_ + config_.damping_duration) {
      decision.command_authority = CommandAuthority::kDampingOnly;
      decision.foc_allowed = true;
    }
    return decision;
  }

  if (arm_requested_ && InputsHealthyLocked(now)) {
    decision.command_authority = CommandAuthority::kActive;
    decision.foc_allowed = true;
    decision.policy_allowed = control_mode_ == ControlMode::kControl;
    return decision;
  }

  decision.enforced_mode = ControlMode::kDamping;
  decision.prearm_pending = arm_requested_;
  decision.reason = arm_requested_ ? "pre-arm safety gates are not ready"
                                   : "controller is not armed";
  return decision;
}

}  // namespace a2

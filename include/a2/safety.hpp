#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>

#include "a2/config.hpp"
#include "a2/contract.hpp"

namespace a2 {

enum class ControlMode {
  kDamping = 0,
  kStand = 1,
  kControl = 2,
};

enum class CommandAuthority {
  kStopOnly = 0,
  kDampingOnly = 1,
  kActive = 2,
};

enum class FaultCode {
  kNone = 0,
  kInvalidLowStateCrc,
  kInitialLowStateTimeout,
  kLowStateTimeout,
  kTickNotAdvancing,
  kUnexpectedModeMachine,
  kNonFiniteState,
  kQuaternionNorm,
  kExcessiveTilt,
  kJointPositionLimit,
  kJointVelocityLimit,
  kMainboardFault,
  kMainboardPrearmTimeout,
  kMainboardRuntimeTimeout,
  kMotionCheckFailed,
  kUnexpectedForm,
  kMotionModeNotReleased,
  kMotionReleaseTimeout,
  kExternalFault,
};

const char* FaultCodeName(FaultCode fault) noexcept;

struct LowStateSnapshot {
  bool crc_valid{false};
  std::uint32_t tick{0};
  std::uint8_t mode_machine{0};
  // Unitree HG convention: [w, x, y, z].
  std::array<float, 4> quaternion{1.0F, 0.0F, 0.0F, 0.0F};
  Vec3 body_angular_velocity{};
  JointArray joint_position{};
  JointArray joint_velocity{};
};

struct SafetyDecision {
  CommandAuthority command_authority{CommandAuthority::kStopOnly};
  bool foc_allowed{false};
  bool policy_allowed{false};
  bool armed{false};
  bool fault_latched{false};
  bool prearm_pending{false};
  FaultCode fault{FaultCode::kNone};
  ControlMode enforced_mode{ControlMode::kDamping};
  std::string reason;
  std::chrono::milliseconds low_state_age{0};
  std::chrono::milliseconds mainboard_age{0};
};

class SafetySupervisor {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit SafetySupervisor(const A2Config& config,
                            TimePoint start_time = Clock::now(),
                            float joint_position_tolerance_rad = 0.0F);

  SafetySupervisor(const SafetySupervisor&) = delete;
  SafetySupervisor& operator=(const SafetySupervisor&) = delete;

  // A CRC failure or unsafe state is latched immediately. Duplicate/backward
  // ticks are inspected but do not refresh the last-valid-state timestamp.
  void ObserveLowState(const LowStateSnapshot& state, TimePoint now);
  void ObserveMainboard(std::uint32_t state0, TimePoint now);
  void ObserveMotionStatus(bool check_succeeded, const std::string& form,
                           const std::string& mode, bool released,
                           TimePoint now);

  // START requests pre-arm. FOC remains blocked until every input gate is
  // healthy. Mode transitions are DAMPING -> STAND -> CONTROL; DAMPING is
  // always a valid downward transition.
  bool RequestArm(TimePoint now);
  void Disarm();
  bool RequestMode(ControlMode requested);

  // Faults remain latched even after new healthy samples arrive. An explicit
  // L1+START-equivalent rearm can clear the latch only after all gates recover
  // and the configured damping+STOP interval has elapsed. Rearm always
  // returns to DAMPING.
  bool ExplicitRearm(TimePoint now);
  void ForceFault(FaultCode fault, const std::string& reason, TimePoint now);

  SafetyDecision Evaluate(TimePoint now);

  bool fault_latched() const;
  bool armed() const;
  ControlMode control_mode() const;

  static float QuaternionTiltRad(
      const std::array<float, 4>& quaternion) noexcept;

 private:
  bool LowStateHealthyLocked(TimePoint now) const;
  bool CurrentMainboardHealthyLocked(TimePoint now) const;
  bool MainboardHealthyLocked(TimePoint now) const;
  bool MotionHealthyLocked() const;
  bool InputsHealthyLocked(TimePoint now) const;
  bool InputsRecoveredLocked(TimePoint now) const;
  void LatchFaultLocked(FaultCode fault, const std::string& reason,
                        TimePoint now);
  SafetyDecision BuildDecisionLocked(TimePoint now) const;

  A2Config config_;
  // Zero on hardware. A small nonzero value is permitted only when the
  // controller explicitly constructs the supervisor for simulation.
  float joint_position_tolerance_rad_{0.0F};
  mutable std::mutex mutex_;
  TimePoint start_time_{};
  TimePoint prearm_start_time_{};
  TimePoint last_low_state_receipt_time_{};
  TimePoint last_valid_low_state_time_{};
  TimePoint last_tick_progress_time_{};
  TimePoint last_mainboard_time_{};
  TimePoint last_motion_status_time_{};
  TimePoint fault_time_{};

  bool arm_requested_{false};
  bool ever_foc_allowed_{false};
  bool have_valid_low_state_{false};
  bool have_low_state_receipt_{false};
  bool latest_low_state_safe_{false};
  bool have_tick_{false};
  bool have_mainboard_{false};
  bool latest_mainboard_safe_{false};
  bool mainboard_arm_gate_satisfied_{false};
  bool have_motion_status_{false};
  bool motion_check_succeeded_{false};
  bool motion_released_{false};
  std::uint32_t last_tick_{0};
  std::uint32_t mainboard_state0_{0};
  std::uint64_t mainboard_generation_{0};
  std::uint64_t arm_mainboard_generation_{0};
  std::string motion_form_;
  std::string motion_mode_;

  ControlMode control_mode_{ControlMode::kDamping};
  FaultCode fault_{FaultCode::kNone};
  std::string fault_reason_;
};

}  // namespace a2

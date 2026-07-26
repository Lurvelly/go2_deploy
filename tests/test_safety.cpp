#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <string>

#include "a2/config.hpp"
#include "a2/contract.hpp"
#include "a2/safety.hpp"

namespace {

int failures = 0;

void Check(const bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

using TimePoint = a2::SafetySupervisor::TimePoint;

TimePoint At(const long long milliseconds) {
  return TimePoint{} + std::chrono::milliseconds(milliseconds);
}

a2::LowStateSnapshot SafeState(const std::uint32_t tick = 1U) {
  a2::LowStateSnapshot state;
  state.crc_valid = true;
  state.tick = tick;
  state.mode_machine = a2::kExpectedModeMachine;
  state.quaternion = {1.0F, 0.0F, 0.0F, 0.0F};
  state.body_angular_velocity.fill(0.0F);
  state.joint_position = a2::kDefaultPositions;
  state.joint_velocity.fill(0.0F);
  return state;
}

void ObserveLowAndMotion(a2::SafetySupervisor& safety, const TimePoint now,
                         const std::uint32_t tick) {
  safety.ObserveLowState(SafeState(tick), now);
  safety.ObserveMotionStatus(true, "0", "", true, now);
}

void ArmWithFreshZero(a2::SafetySupervisor& safety, const TimePoint arm_at,
                      const TimePoint ready_at, const std::uint32_t tick) {
  ObserveLowAndMotion(safety, arm_at, tick);
  safety.ObserveMainboard(0U, arm_at);  // Deliberately predates START.
  Check(safety.RequestArm(arm_at), "fixture START request is accepted");
  safety.ObserveLowState(SafeState(tick + 1U), ready_at);
  safety.ObserveMainboard(0U, ready_at);  // New generation opens the gate.
}

void TestPostArmGenerationGateAndInitialTimeout() {
  const a2::A2Config config;
  a2::SafetySupervisor safety(config, At(0));
  ObserveLowAndMotion(safety, At(0), 1U);
  safety.ObserveMainboard(0U, At(0));
  Check(safety.RequestArm(At(1)), "initial START request is accepted");

  auto decision = safety.Evaluate(At(1));
  Check(!decision.foc_allowed && decision.prearm_pending &&
            decision.command_authority == a2::CommandAuthority::kStopOnly,
        "a zero MainBoardState from before START cannot authorize FOC");
  Check(!safety.RequestMode(a2::ControlMode::kStand),
        "STAND cannot be queued while post-START zero is pending");

  safety.ObserveLowState(SafeState(2U), At(2));
  safety.ObserveMainboard(0U, At(2));
  decision = safety.Evaluate(At(2));
  Check(decision.foc_allowed &&
            decision.command_authority == a2::CommandAuthority::kActive,
        "a later zero MainBoardState generation opens the FOC gate");
  Check(!decision.policy_allowed &&
            decision.enforced_mode == a2::ControlMode::kDamping,
        "arming starts in DAMPING rather than policy control");

  a2::SafetySupervisor missing(config, At(0));
  decision = missing.Evaluate(At(5001));
  Check(decision.fault_latched &&
            decision.fault == a2::FaultCode::kInitialLowStateTimeout,
        "missing initial LowState latches after the startup deadline");
}

void TestWatchdogAndFrozenTick() {
  const a2::A2Config config;
  a2::SafetySupervisor timeout(config, At(0));
  ArmWithFreshZero(timeout, At(0), At(1), 10U);
  Check(timeout.Evaluate(At(101)).foc_allowed,
        "100 ms LowState boundary remains valid");
  const auto timed_out = timeout.Evaluate(At(102));
  Check(timed_out.fault == a2::FaultCode::kLowStateTimeout &&
            timed_out.command_authority == a2::CommandAuthority::kStopOnly,
        "a stale LowState latches and revokes FOC immediately");

  a2::SafetySupervisor frozen(config, At(0));
  ArmWithFreshZero(frozen, At(0), At(1), 50U);
  Check(frozen.Evaluate(At(1)).foc_allowed,
        "frozen-tick fixture first becomes active");
  frozen.ObserveLowState(SafeState(51U), At(1));
  frozen.ObserveLowState(SafeState(51U), At(102));
  const auto frozen_decision = frozen.Evaluate(At(102));
  Check(frozen_decision.fault == a2::FaultCode::kTickNotAdvancing &&
            frozen_decision.command_authority ==
                a2::CommandAuthority::kStopOnly,
        "duplicate ticks do not refresh the trusted-state watchdog");
}

void TestStateValidationAndStopOnlyFaults() {
  const a2::A2Config config;
  a2::SafetySupervisor crc(config, At(0));
  ArmWithFreshZero(crc, At(0), At(1), 1U);
  Check(crc.Evaluate(At(1)).foc_allowed, "CRC fixture reaches FOC");
  auto state = SafeState(3U);
  state.crc_valid = false;
  crc.ObserveLowState(state, At(2));
  auto decision = crc.Evaluate(At(2));
  Check(decision.fault == a2::FaultCode::kInvalidLowStateCrc &&
            !decision.foc_allowed &&
            decision.command_authority == a2::CommandAuthority::kStopOnly,
        "invalid CRC is an immediate STOP-only fault after FOC");

  a2::SafetySupervisor mode(config, At(0));
  state = SafeState();
  state.mode_machine = 0U;
  mode.ObserveLowState(state, At(0));
  Check(mode.Evaluate(At(0)).fault ==
            a2::FaultCode::kUnexpectedModeMachine,
        "unexpected mode_machine fails closed");

  a2::SafetySupervisor gyro(config, At(0));
  state = SafeState();
  state.body_angular_velocity[1] =
      std::numeric_limits<float>::quiet_NaN();
  gyro.ObserveLowState(state, At(0));
  Check(gyro.Evaluate(At(0)).fault == a2::FaultCode::kNonFiniteState,
        "non-finite policy gyro input fails closed");

  a2::SafetySupervisor tilt(config, At(0));
  state = SafeState();
  constexpr float half_angle = 0.6F;
  state.quaternion = {std::cos(half_angle), std::sin(half_angle), 0.0F,
                      0.0F};
  tilt.ObserveLowState(state, At(0));
  Check(tilt.Evaluate(At(0)).fault == a2::FaultCode::kExcessiveTilt,
        "excessive yaw-invariant body tilt latches");

  a2::SafetySupervisor quaternion(config, At(0));
  state = SafeState();
  state.quaternion = {2.0F, 0.0F, 0.0F, 0.0F};
  quaternion.ObserveLowState(state, At(0));
  Check(quaternion.Evaluate(At(0)).fault ==
            a2::FaultCode::kQuaternionNorm,
        "invalid quaternion norm latches");

  a2::SafetySupervisor position(config, At(0));
  state = SafeState();
  state.joint_position[8] = a2::kHardUpper[8] + 0.01F;
  position.ObserveLowState(state, At(0));
  Check(position.Evaluate(At(0)).fault ==
            a2::FaultCode::kJointPositionLimit,
        "hard joint position violation latches");

  state = SafeState();
  state.joint_position[0] = a2::kHardUpper[0] + 0.003F;
  a2::SafetySupervisor hardware_limits(config, At(0));
  hardware_limits.ObserveLowState(state, At(0));
  Check(hardware_limits.Evaluate(At(0)).fault ==
            a2::FaultCode::kJointPositionLimit,
        "hardware keeps strict hard joint limits for milliradian overshoot");

  a2::SafetySupervisor simulation_limits(
      config, At(0), config.sim_joint_position_tolerance_rad);
  simulation_limits.ObserveLowState(state, At(0));
  Check(simulation_limits.Evaluate(At(0)).fault == a2::FaultCode::kNone,
        "simulation-only tolerance accepts bounded solver overshoot");
  state.tick = 2U;
  state.joint_position[0] = a2::kHardUpper[0] +
                            config.sim_joint_position_tolerance_rad + 0.001F;
  simulation_limits.ObserveLowState(state, At(1));
  Check(simulation_limits.Evaluate(At(1)).fault ==
            a2::FaultCode::kJointPositionLimit,
        "simulation tolerance remains bounded and fail-closed");

  a2::SafetySupervisor velocity(config, At(0));
  state = SafeState();
  state.joint_velocity[2] = a2::kVelocityLimits[2] + 0.01F;
  velocity.ObserveLowState(state, At(0));
  Check(velocity.Evaluate(At(0)).fault ==
            a2::FaultCode::kJointVelocityLimit,
        "joint velocity violation latches");
}

void TestMainboardGateAndRuntimeFaults() {
  const a2::A2Config config;
  for (const std::uint32_t bit : {1U << 12U, 1U << 23U}) {
    a2::SafetySupervisor mainboard(config, At(0));
    mainboard.ObserveMainboard(bit, At(0));
    const auto decision = mainboard.Evaluate(At(0));
    Check(decision.fault == a2::FaultCode::kMainboardFault,
          "32-bit MainBoardState fault bits are not truncated");
  }

  a2::SafetySupervisor pending(config, At(0));
  ObserveLowAndMotion(pending, At(0), 1U);
  pending.ObserveMainboard(0U, At(0));
  pending.RequestArm(At(1));
  pending.ObserveMainboard(1U, At(2));
  auto decision = pending.Evaluate(At(2));
  Check(!decision.fault_latched && !decision.foc_allowed,
        "only bit 0 may remain pending during the first pre-arm");
  pending.ObserveLowState(SafeState(2U), At(100));
  pending.ObserveMainboard(0U, At(100));
  Check(pending.Evaluate(At(100)).foc_allowed,
        "a later zero completes first pre-arm");

  a2::SafetySupervisor stuck(config, At(0));
  ObserveLowAndMotion(stuck, At(0), 1U);
  stuck.RequestArm(At(0));
  stuck.ObserveMainboard(1U, At(1));
  stuck.ObserveLowState(SafeState(2U), At(2001));
  Check(stuck.Evaluate(At(2001)).fault ==
            a2::FaultCode::kMainboardPrearmTimeout,
        "pre-arm bit 0 must clear within two seconds");

  a2::SafetySupervisor runtime_bit(config, At(0));
  ArmWithFreshZero(runtime_bit, At(0), At(1), 1U);
  Check(runtime_bit.Evaluate(At(1)).foc_allowed,
        "runtime mainboard fixture reaches FOC");
  runtime_bit.ObserveMainboard(1U, At(2));
  decision = runtime_bit.Evaluate(At(2));
  Check(decision.fault == a2::FaultCode::kMainboardFault &&
            decision.command_authority == a2::CommandAuthority::kStopOnly,
        "any nonzero MainBoardState is STOP-only after FOC authority");

  a2::SafetySupervisor runtime_timeout(config, At(0));
  ArmWithFreshZero(runtime_timeout, At(0), At(1), 10U);
  Check(runtime_timeout.Evaluate(At(1)).foc_allowed,
        "runtime-timeout fixture reaches FOC");
  runtime_timeout.ObserveLowState(SafeState(12U), At(2002));
  decision = runtime_timeout.Evaluate(At(2002));
  Check(decision.fault == a2::FaultCode::kMainboardRuntimeTimeout &&
            decision.command_authority == a2::CommandAuthority::kStopOnly,
        "stale runtime MainBoardState revokes FOC immediately");

  a2::SafetySupervisor absent(config, At(0));
  ObserveLowAndMotion(absent, At(0), 1U);
  absent.RequestArm(At(0));
  absent.ObserveLowState(SafeState(2U), At(2001));
  Check(absent.Evaluate(At(2001)).fault ==
            a2::FaultCode::kMainboardPrearmTimeout,
        "no post-arm MainBoardState times out as pre-arm, not runtime");
}

void TestMotionAuthorityAndTransitions() {
  const a2::A2Config config;
  a2::SafetySupervisor motion(config, At(0));
  ArmWithFreshZero(motion, At(0), At(1), 1U);
  Check(motion.Evaluate(At(1)).foc_allowed,
        "motion fixture reaches FOC");
  motion.ObserveMotionStatus(true, "0", "ai_sport", false, At(2));
  auto decision = motion.Evaluate(At(2));
  Check(decision.fault == a2::FaultCode::kMotionModeNotReleased &&
            decision.command_authority == a2::CommandAuthority::kStopOnly,
        "a runtime motion mode conflict is an immediate STOP-only fault");

  a2::SafetySupervisor check_failed(config, At(0));
  check_failed.ObserveMotionStatus(false, "", "", false, At(0));
  Check(check_failed.Evaluate(At(0)).fault ==
            a2::FaultCode::kMotionCheckFailed,
        "MotionSwitcher RPC failure latches immediately");

  a2::SafetySupervisor bad_form(config, At(0));
  bad_form.ObserveMotionStatus(true, "1", "", true, At(0));
  Check(bad_form.Evaluate(At(0)).fault == a2::FaultCode::kUnexpectedForm,
        "non-standard A2 form latches immediately");

  a2::SafetySupervisor transitions(config, At(0));
  ArmWithFreshZero(transitions, At(0), At(1), 20U);
  Check(!transitions.RequestMode(a2::ControlMode::kStand),
        "mode elevation waits until active authority is evaluated");
  Check(transitions.Evaluate(At(1)).foc_allowed,
        "transition fixture obtains active authority");
  Check(!transitions.RequestMode(a2::ControlMode::kControl),
        "DAMPING cannot skip directly to CONTROL");
  Check(transitions.RequestMode(a2::ControlMode::kStand),
        "DAMPING transitions to STAND");
  Check(transitions.RequestMode(a2::ControlMode::kControl),
        "STAND transitions to CONTROL");
  Check(transitions.Evaluate(At(1)).policy_allowed,
        "policy is authorized only in CONTROL");
  Check(transitions.RequestMode(a2::ControlMode::kDamping),
        "any active state can return to DAMPING");
}

void TestFaultLatchAndGenerationBasedRearm() {
  const a2::A2Config config;
  a2::SafetySupervisor safety(config, At(0));
  ArmWithFreshZero(safety, At(0), At(1), 1U);
  Check(safety.Evaluate(At(1)).foc_allowed,
        "rearm fixture reaches active FOC");
  safety.ForceFault(a2::FaultCode::kExternalFault, "test fault", At(10));

  auto decision = safety.Evaluate(At(10));
  Check(decision.fault_latched && decision.foc_allowed &&
            decision.command_authority ==
                a2::CommandAuthority::kDampingOnly,
        "healthy-state external fault receives bounded damping authority");
  Check(!safety.ExplicitRearm(At(500)),
        "rearm is refused before damping plus STOP completes");

  ObserveLowAndMotion(safety, At(610), 3U);
  safety.ObserveMainboard(0U, At(610));
  Check(safety.ExplicitRearm(At(610)),
        "rearm request accepts recovered current inputs");
  decision = safety.Evaluate(At(610));
  Check(!decision.fault_latched && !decision.foc_allowed &&
            decision.prearm_pending &&
            decision.command_authority == a2::CommandAuthority::kStopOnly,
        "rearm remains STOP-only until another mainboard generation");
  Check(!safety.RequestMode(a2::ControlMode::kStand),
        "STAND cannot be queued during rearm pre-arm");

  safety.ObserveLowState(SafeState(4U), At(611));
  safety.ObserveMainboard(0U, At(611));
  decision = safety.Evaluate(At(611));
  Check(decision.foc_allowed && !decision.policy_allowed &&
            decision.enforced_mode == a2::ControlMode::kDamping,
        "post-rearm zero returns only to DAMPING authority");
}

void TestExternalDampingStopsOnMainboardFault() {
  const a2::A2Config config;
  a2::SafetySupervisor safety(config, At(0));
  ArmWithFreshZero(safety, At(0), At(1), 1U);
  Check(safety.Evaluate(At(1)).foc_allowed,
        "mainboard-revocation fixture reaches FOC");
  safety.ForceFault(a2::FaultCode::kExternalFault, "test fault", At(10));
  Check(safety.Evaluate(At(10)).command_authority ==
            a2::CommandAuthority::kDampingOnly,
        "external fault initially receives bounded damping");

  safety.ObserveMainboard(1U, At(11));
  const auto decision = safety.Evaluate(At(11));
  Check(!decision.foc_allowed &&
            decision.command_authority == a2::CommandAuthority::kStopOnly,
        "nonzero MainBoardState revokes external-fault damping immediately");
}

void TestExternalDampingStopsOnMotionConflict() {
  const a2::A2Config config;
  a2::SafetySupervisor safety(config, At(0));
  ArmWithFreshZero(safety, At(0), At(1), 1U);
  Check(safety.Evaluate(At(1)).foc_allowed,
        "motion-revocation fixture reaches FOC");
  safety.ForceFault(a2::FaultCode::kExternalFault, "test fault", At(10));
  Check(safety.Evaluate(At(10)).command_authority ==
            a2::CommandAuthority::kDampingOnly,
        "external fault initially retains healthy motion authority");

  safety.ObserveMotionStatus(true, "0", "ai_sports", false, At(11));
  const auto decision = safety.Evaluate(At(11));
  Check(!decision.foc_allowed &&
            decision.command_authority == a2::CommandAuthority::kStopOnly,
        "MotionSwitcher conflict revokes external-fault damping immediately");
}

void TestRearmWithoutNewMainboardGenerationTimesOut() {
  const a2::A2Config config;
  a2::SafetySupervisor safety(config, At(0));
  ArmWithFreshZero(safety, At(0), At(1), 1U);
  Check(safety.Evaluate(At(1)).foc_allowed,
        "rearm-timeout fixture reaches FOC");
  safety.ForceFault(a2::FaultCode::kExternalFault, "test fault", At(10));

  ObserveLowAndMotion(safety, At(610), 3U);
  safety.ObserveMainboard(0U, At(610));
  Check(safety.ExplicitRearm(At(610)),
        "rearm-timeout fixture accepts recovered inputs");
  safety.ObserveLowState(SafeState(4U), At(2609));
  Check(safety.Evaluate(At(2609)).prearm_pending,
        "rearm remains pending before the two-second deadline");

  safety.ObserveLowState(SafeState(5U), At(2610));
  const auto decision = safety.Evaluate(At(2610));
  Check(decision.fault == a2::FaultCode::kMainboardPrearmTimeout &&
            decision.command_authority == a2::CommandAuthority::kStopOnly,
        "rearm without a later MainBoardState generation times out at two seconds");
}

}  // namespace

int main() {
  TestPostArmGenerationGateAndInitialTimeout();
  TestWatchdogAndFrozenTick();
  TestStateValidationAndStopOnlyFaults();
  TestMainboardGateAndRuntimeFaults();
  TestMotionAuthorityAndTransitions();
  TestFaultLatchAndGenerationBasedRearm();
  TestExternalDampingStopsOnMainboardFault();
  TestExternalDampingStopsOnMotionConflict();
  TestRearmWithoutNewMainboardGenerationTimesOut();
  if (failures != 0) {
    std::cerr << failures << " safety test(s) failed\n";
    return 1;
  }
  std::cout << "A2 safety tests passed\n";
  return 0;
}

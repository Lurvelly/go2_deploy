#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

#include "a2/contract.hpp"
#include "a2/crc32.hpp"
#include "a2/low_command.hpp"
#include "a2/policy_deadline.hpp"
#include "a2/safety.hpp"

namespace {

int failures = 0;

void Check(const bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

a2::LowCommandBuildInput CommandInput(const a2::LowCommandKind kind) {
  a2::LowCommandBuildInput input;
  input.kind = kind;
  input.mode_machine = a2::kExpectedModeMachine;
  for (std::size_t i = 0; i < a2::kNumJoints; ++i) {
    input.motor_indices[i] = i;
    input.joint_position[i] = a2::kDefaultPositions[i];
    input.joint_velocity[i] = static_cast<float>(i + 1U);
    input.target_position[i] = a2::kDefaultPositions[i] + 0.01F;
    input.kp[i] = a2::kPositionKp[i];
    input.kd[i] = a2::kPositionKd[i];
    input.effort_limits[i] = a2::kEffortLimits[i];
  }
  return input;
}

template <typename MotorCommand>
bool IsStopSlot(const MotorCommand& motor) {
  return motor.mode() == 0U && motor.q() == 0.0F && motor.dq() == 0.0F &&
         motor.tau() == 0.0F && motor.kp() == 0.0F &&
         motor.kd() == 0.0F && motor.reserve() == 0U;
}

void CheckEnvelope(const unitree_hg::msg::dds_::LowCmd_& command,
                   const std::string& context) {
  Check(command.motor_cmd().size() == a2::kMotorSlots,
        context + ": command contains all 35 HG motor slots");
  Check(command.mode_pr() == 0U, context + ": mode_pr is PR mode 0");
  Check(command.mode_machine() == a2::kExpectedModeMachine,
        context + ": mode_machine mirrors LowState");
  for (const std::uint32_t value : command.reserve()) {
    Check(value == 0U, context + ": LowCmd reserve remains zero");
  }
  Check(command.crc() == a2::ComputeMessageCrc(command),
        context + ": CRC covers the completed command");
}

void TestStopAndNoResidualFoc() {
  auto input = CommandInput(a2::LowCommandKind::kPosition);
  const auto foc = a2::BuildLowCommand(input);
  CheckEnvelope(foc, "position");
  for (std::size_t i = 0; i < a2::kNumJoints; ++i) {
    Check(foc.motor_cmd()[i].mode() == 0x01U,
          "position: policy motor is in FOC mode");
  }
  for (std::size_t i = a2::kNumJoints; i < a2::kMotorSlots; ++i) {
    Check(IsStopSlot(foc.motor_cmd()[i]),
          "position: unused HG slot remains STOP");
  }

  input.kind = a2::LowCommandKind::kStop;
  const auto stop = a2::BuildLowCommand(input);
  CheckEnvelope(stop, "stop after FOC");
  for (const auto& motor : stop.motor_cmd()) {
    Check(IsStopSlot(motor), "stop after FOC: no motor field is retained");
  }
}

void TestDampingAndUnusedSlots() {
  const auto input = CommandInput(a2::LowCommandKind::kDamping);
  const auto command = a2::BuildLowCommand(input);
  CheckEnvelope(command, "damping");
  for (std::size_t i = 0; i < a2::kNumJoints; ++i) {
    const auto& motor = command.motor_cmd()[i];
    Check(motor.mode() == 0x01U, "damping: policy motor is in FOC mode");
    Check(motor.q() == input.joint_position[i],
          "damping: q follows the current joint position");
    Check(motor.kp() == 0.0F, "damping: kp is zero");
    Check(motor.kd() >= 0.0F &&
              motor.kd() * std::abs(input.joint_velocity[i]) <=
                  input.effort_limits[i] + 1.0e-5F,
          "damping: derivative effort is limited");
  }
  for (std::size_t i = a2::kNumJoints; i < a2::kMotorSlots; ++i) {
    Check(IsStopSlot(command.motor_cmd()[i]),
          "damping: slots 12..34 remain STOP");
  }
}

void TestFailClosedAuthority() {
  a2::SafetyDecision decision;
  decision.foc_allowed = false;
  decision.command_authority = a2::CommandAuthority::kStopOnly;
  Check(!a2::FaultDampingAuthorized(decision, true, true),
        "fault: STOP-only authority cannot be upgraded to FOC");
  Check(!a2::ShutdownDampingAuthorized(decision, true, true),
        "shutdown: unarmed STOP-only authority cannot be upgraded to FOC");

  decision.foc_allowed = true;
  decision.command_authority = a2::CommandAuthority::kDampingOnly;
  Check(a2::FaultDampingAuthorized(decision, true, true),
        "fault: supervisor damping authority is honored");
  Check(!a2::FaultDampingAuthorized(decision, false, true),
        "fault: stale state blocks damping");
  Check(!a2::FaultDampingAuthorized(decision, true, false),
        "fault: no prior FOC publication blocks damping");

  decision.command_authority = a2::CommandAuthority::kActive;
  Check(!a2::FaultDampingAuthorized(decision, true, true),
        "fault: active authority is not accepted for a latched fault");
  Check(a2::ShutdownDampingAuthorized(decision, true, true),
        "shutdown: active authority permits bounded damping");
}

void TestPolicyDeadlineWatchdog() {
  a2::PolicyDeadlineWatchdog watchdog;

  auto decision = watchdog.Observe(20.0, 20.0);
  Check(!decision.missed && !decision.fatal && decision.accept_result,
        "policy deadline: exact soft deadline is accepted");

  decision = watchdog.Observe(21.0762, 20.0);
  Check(decision.missed && !decision.fatal && decision.accept_result &&
            decision.consecutive_misses == 1,
        "policy deadline: one small scheduler overrun is tolerated");
  decision = watchdog.Observe(19.0, 20.0);
  Check(!decision.missed && watchdog.consecutive_misses() == 0,
        "policy deadline: an on-time inference clears the miss streak");

  decision = watchdog.Observe(21.0, 20.0);
  Check(!decision.fatal && decision.accept_result,
        "policy deadline: first consecutive miss remains usable");
  decision = watchdog.Observe(22.0, 20.0);
  Check(!decision.fatal && decision.accept_result &&
            decision.consecutive_misses == 2,
        "policy deadline: second consecutive miss remains usable");
  decision = watchdog.Observe(23.0, 20.0);
  Check(decision.fatal && !decision.accept_result &&
            decision.consecutive_misses == 3,
        "policy deadline: third consecutive miss fails closed");

  watchdog.Reset();
  decision = watchdog.Observe(40.0, 20.0);
  Check(decision.fatal && !decision.accept_result &&
            decision.consecutive_misses == 1,
        "policy deadline: one two-period inference is a hard miss");
  watchdog.Reset();
  Check(watchdog.consecutive_misses() == 0,
        "policy deadline: explicit reset clears the miss streak");
}

}  // namespace

int main() {
  TestStopAndNoResidualFoc();
  TestDampingAndUnusedSlots();
  TestFailClosedAuthority();
  TestPolicyDeadlineWatchdog();
  if (failures != 0) {
    std::cerr << failures << " low-command test(s) failed\n";
    return 1;
  }
  std::cout << "A2 low-command tests passed\n";
  return 0;
}

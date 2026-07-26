#include "a2/controller.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include "a2/contract.hpp"
#include "a2/crc32.hpp"
#include "a2/policy.hpp"
#include "a2/safety.hpp"
#include "a2/low_command.hpp"
#include "a2/navigation.hpp"
#include "a2/policy_deadline.hpp"

#include <unitree/dds_wrapper/common/unitree_joystick.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/MainBoardState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace a2 {
namespace {

using Clock = std::chrono::steady_clock;
using LowCmd = unitree_hg::msg::dds_::LowCmd_;
using LowState = unitree_hg::msg::dds_::LowState_;
using MainBoardState = unitree_hg::msg::dds_::MainBoardState_;

constexpr const char* kLowCommandTopic = "rt/lowcmd";
constexpr const char* kLowStateTopic = "rt/lowstate";
constexpr const char* kMainboardStateTopic = "rt/lf/mainboardstate";

enum class RuntimePhase {
  kPrearm = 0,
  kDamping = 1,
  kStand = 2,
  kControl = 3,
  kFault = 4,
  kStopping = 5,
  kStopped = 6,
};

const char* PhaseName(const RuntimePhase phase) noexcept {
  switch (phase) {
    case RuntimePhase::kPrearm:
      return "PREARM";
    case RuntimePhase::kDamping:
      return "DAMPING";
    case RuntimePhase::kStand:
      return "STAND";
    case RuntimePhase::kControl:
      return "CTRL";
    case RuntimePhase::kFault:
      return "FAULT";
    case RuntimePhase::kStopping:
      return "STOPPING";
    case RuntimePhase::kStopped:
      return "STOPPED";
  }
  return "UNKNOWN";
}

bool TickAdvanced(const std::uint32_t previous,
                  const std::uint32_t current) noexcept {
  return static_cast<std::int32_t>(current - previous) > 0;
}

template <std::size_t N>
bool Finite(const std::array<float, N>& values) noexcept {
  return std::all_of(values.begin(), values.end(),
                     [](const float value) { return std::isfinite(value); });
}

Vec3 ProjectGravity(const std::array<float, 4>& quaternion) {
  const float norm = std::sqrt(
      quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1] +
      quaternion[2] * quaternion[2] + quaternion[3] * quaternion[3]);
  if (!std::isfinite(norm) || norm <= std::numeric_limits<float>::epsilon()) {
    return {0.0F, 0.0F, -1.0F};
  }
  const float w = quaternion[0] / norm;
  const float x = quaternion[1] / norm;
  const float y = quaternion[2] / norm;
  const float z = quaternion[3] / norm;
  return {
      2.0F * (w * y - x * z),
      -2.0F * (y * z + w * x),
      2.0F * (x * x + y * y) - 1.0F,
  };
}

std::string CsvSafe(std::string text) {
  std::replace(text.begin(), text.end(), ',', ';');
  std::replace(text.begin(), text.end(), '\n', ' ');
  std::replace(text.begin(), text.end(), '\r', ' ');
  return text;
}

}  // namespace

class A2Controller::Impl {
 public:
  Impl(A2Config config, RuntimeOptions options)
      : config_(std::move(config)),
        options_(std::move(options)),
        policy_(config_) {
    // The A2 wire format exposes R2 as a button bit. Disable the generic
    // trigger-axis smoothing so L1+R2 mode transitions retain edge semantics.
    gamepad_.RT.smooth = 1.0F;
  }

  ~Impl() {
    RequestShutdown();
    StopThreadsAndDds();
  }

  int Run() {
    if (options_.network_interface.empty()) {
      throw std::invalid_argument("network interface must not be empty");
    }
    if (options_.domain_id < 0 || options_.domain_id > 232) {
      throw std::invalid_argument("DDS domain ID must be in [0, 232]");
    }

    policy_.Load();
    OpenLog();

    try {
      unitree::robot::ChannelFactory::Instance()->Init(
          options_.domain_id, options_.network_interface);
      dds_factory_initialized_ = true;

      if (options_.simulation) {
        last_motion_form_ = config_.expected_form;
        last_motion_mode_.clear();
        std::cout << "Simulation mode: MotionSwitcher RPC is disabled"
                  << std::endl;
      } else {
        motion_switcher_ =
            std::make_unique<unitree::robot::b2::MotionSwitcherClient>();
        ReleaseMotionControl();
      }
      safety_ = std::make_unique<SafetySupervisor>(
          config_, Clock::now(),
          options_.simulation ? config_.sim_joint_position_tolerance_rad
                              : 0.0F);
      safety_->ObserveMotionStatus(true, last_motion_form_,
                                   last_motion_mode_, true, Clock::now());
      InitChannels();

      threads_running_.store(true, std::memory_order_release);
      command_thread_ = std::thread(&Impl::CommandThreadMain, this);
      control_thread_ = std::thread(&Impl::ControlThreadMain, this);

      std::cout
          << "A2 DDS ready on " << options_.network_interface
          << " (domain " << options_.domain_id << ")\n"
          << "Waiting for CRC-valid LowState and healthy MainBoardState."
          << std::endl;
      if (!options_.simulation ||
          options_.navigation_source == NavigationSource::kGamepad) {
        std::cout << "START: arm damping | L1+R2: stand | L1+A: policy | "
                     "L1+Y: damping | L1+START: rearm a cleared fault"
                  << std::endl;
      } else {
        std::cout << "Operator state changes use simulation terminal commands: "
                     "arm, stand, ctrl, damp, rearm."
                  << std::endl;
      }

      while (!shutdown_requested_.load(std::memory_order_acquire)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }

      BeginControlledShutdown();
      const auto wait = config_.damping_duration + config_.stop_duration +
                        std::chrono::milliseconds(50);
      std::this_thread::sleep_for(wait);
      MarkStopped();
      StopThreadsAndDds();
      if (const std::exception_ptr failure = CopyWorkerFailure()) {
        std::rethrow_exception(failure);
      }
      return 0;
    } catch (...) {
      shutdown_requested_.store(true, std::memory_order_release);
      StopThreadsAndDds();
      throw;
    }
  }

  void RequestShutdown() noexcept {
    shutdown_requested_.store(true, std::memory_order_release);
    if (threads_running_.load(std::memory_order_acquire)) {
      try {
        BeginControlledShutdown();
      } catch (...) {
        // Run() will observe shutdown_requested_ and retry the transition.
      }
    }
  }

  void SubmitOperatorRequest(const OperatorRequest request) {
    std::lock_guard<std::mutex> lock(submitted_input_mutex_);
    pending_operator_request_ = request;
  }

  void SubmitNavigationAction(const Vec3& action) {
    if (!Finite(action)) {
      throw std::invalid_argument("navigation action contains NaN or Inf");
    }
    std::lock_guard<std::mutex> lock(submitted_input_mutex_);
    submitted_navigation_action_ = action;
    submitted_navigation_received_ = true;
    submitted_navigation_at_ = Clock::now();
  }

 private:
  struct RobotSnapshot {
    bool received{false};
    std::uint64_t sequence{0};
    std::uint32_t tick{0};
    std::uint8_t mode_machine{0};
    Clock::time_point received_at{};
    std::array<float, 4> quaternion{1.0F, 0.0F, 0.0F, 0.0F};
    Vec3 projected_gravity{0.0F, 0.0F, -1.0F};
    Vec3 angular_velocity{};
    JointArray q{};
    JointArray dq{};
    std::array<std::uint8_t, 40> wireless_remote{};
  };

  struct MainboardSnapshot {
    bool received{false};
    std::uint64_t sequence{0};
    std::uint32_t state0{0};
    Clock::time_point received_at{};
  };

  struct DesiredCommand {
    RuntimePhase phase{RuntimePhase::kPrearm};
    Clock::time_point phase_started{Clock::now()};
    Clock::time_point shutdown_started{};
    std::uint64_t revision{0};
    JointArray stand_start{};
    // The 50 Hz policy/stand request and the last 500 Hz transmitted target
    // are deliberately separate. The command limiter may need several 2 ms
    // ticks to reach one policy request.
    JointArray requested_q{};
    JointArray sent_q{};
    Vec3 velocity_command{};
    PolicyResult policy_result{};
    bool stand_complete{false};
    std::size_t limiter_count{0};
    double policy_time_ms{0.0};
  };

  void OpenLog() {
    if (options_.log_file.empty()) {
      return;
    }
    std::filesystem::create_directories(options_.log_file.parent_path());
    log_.open(options_.log_file);
    if (!log_) {
      throw std::runtime_error("Cannot open runtime log: " +
                               options_.log_file.string());
    }
    log_ << "time_ms,phase,fault,reason,low_state_age_ms,mainboard_age_ms,"
            "mainboard_state0,cmd_vx,cmd_vy,cmd_yaw,gravity_x,gravity_y,"
            "gravity_z,gyro_x,gyro_y,gyro_z,";
    for (std::size_t i = 0; i < kNumJoints; ++i) log_ << "q" << i << ',';
    for (std::size_t i = 0; i < kNumJoints; ++i) log_ << "dq" << i << ',';
    for (std::size_t i = 0; i < kNumJoints; ++i)
      log_ << "action" << i << ',';
    for (std::size_t i = 0; i < kNumJoints; ++i)
      log_ << "requested_target" << i << ',';
    for (std::size_t i = 0; i < kNumJoints; ++i)
      log_ << "sent_target" << i << ',';
    log_ << "limiter_count,policy_time_ms\n";
    runtime_started_ = Clock::now();
  }

  void ReleaseMotionControl() {
    if (!motion_switcher_) {
      throw std::logic_error("MotionSwitcher client is not initialized");
    }
    motion_switcher_->SetTimeout(5.0F);
    motion_switcher_->Init();
    const auto deadline = Clock::now() + config_.mode_release_timeout;
    while (!shutdown_requested_.load(std::memory_order_acquire) &&
           Clock::now() < deadline) {
      std::string form;
      std::string mode;
      const std::int32_t check = motion_switcher_->CheckMode(form, mode);
      if (check != 0) {
        std::cerr << "MotionSwitcher CheckMode failed: " << check << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }
      if (form != config_.expected_form) {
        throw std::runtime_error(
            "Refusing non-standard A2 form '" + form +
            "' (A2W and unknown forms are unsupported)");
      }
      if (mode.empty()) {
        last_motion_form_ = form;
        last_motion_mode_.clear();
        // Runtime checks run off the 500 Hz publisher, but they still need a
        // bounded response time so a newly active motion service is detected
        // promptly instead of retaining command authority for several seconds.
        motion_switcher_->SetTimeout(0.2F);
        std::cout << "Motion control mode released (form=" << form << ")"
                  << std::endl;
        return;
      }

      std::cout << "Releasing active motion mode '" << mode << "'"
                << std::endl;
      const std::int32_t release = motion_switcher_->ReleaseMode();
      if (release != 0) {
        std::cerr << "MotionSwitcher ReleaseMode failed: " << release
                  << std::endl;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (shutdown_requested_.load(std::memory_order_acquire)) {
      throw std::runtime_error("Shutdown requested while releasing motion mode");
    }
    throw std::runtime_error("Timed out releasing the A2 motion control mode");
  }

  bool MotionStillReleased() noexcept {
    try {
      if (!motion_switcher_) {
        if (safety_) {
          safety_->ForceFault(FaultCode::kMotionCheckFailed,
                              "MotionSwitcher client is not initialized",
                              Clock::now());
        }
        return false;
      }
      std::string form;
      std::string mode;
      const std::int32_t result = motion_switcher_->CheckMode(form, mode);
      const auto checked_at = Clock::now();
      next_motion_check_ = checked_at + std::chrono::seconds(1);
      const bool released = result == 0 && form == config_.expected_form &&
                            mode.empty();
      last_motion_form_ = form;
      last_motion_mode_ = mode;
      if (safety_) {
        safety_->ObserveMotionStatus(result == 0, form, mode, released,
                                     checked_at);
        if (result == 0 && form == config_.expected_form && !mode.empty()) {
          safety_->ForceFault(FaultCode::kMotionModeNotReleased,
                              "MotionSwitcher mode became active: " + mode,
                              checked_at);
        }
      }
      return released;
    } catch (const std::exception& error) {
      next_motion_check_ = Clock::now() + std::chrono::seconds(1);
      try {
        if (safety_) {
          safety_->ForceFault(
              FaultCode::kMotionCheckFailed,
              std::string("MotionSwitcher CheckMode threw: ") + error.what(),
              Clock::now());
        }
      } catch (...) {
      }
      return false;
    } catch (...) {
      next_motion_check_ = Clock::now() + std::chrono::seconds(1);
      try {
        if (safety_) {
          safety_->ForceFault(FaultCode::kMotionCheckFailed,
                              "MotionSwitcher CheckMode threw", Clock::now());
        }
      } catch (...) {
      }
      return false;
    }
  }

  bool MotionAuthorityReady() noexcept {
    if (!options_.simulation) {
      return MotionStillReleased();
    }
    const auto checked_at = Clock::now();
    last_motion_form_ = config_.expected_form;
    last_motion_mode_.clear();
    if (safety_) {
      safety_->ObserveMotionStatus(true, last_motion_form_, last_motion_mode_,
                                   true, checked_at);
    }
    return true;
  }

  void InitChannels() {
    low_command_publisher_ =
        std::make_shared<unitree::robot::ChannelPublisher<LowCmd>>(
            kLowCommandTopic);
    low_state_subscriber_ =
        std::make_shared<unitree::robot::ChannelSubscriber<LowState>>(
            kLowStateTopic);
    mainboard_subscriber_ =
        std::make_shared<unitree::robot::ChannelSubscriber<MainBoardState>>(
            kMainboardStateTopic);

    low_command_publisher_->InitChannel();
    low_state_subscriber_->InitChannel(
        [this](const void* message) {
          try {
            OnLowState(message);
          } catch (...) {
            RecordWorkerFailure("LowState DDS callback");
          }
        },
        1);
    mainboard_subscriber_->InitChannel(
        [this](const void* message) {
          try {
            OnMainboardState(message);
          } catch (...) {
            RecordWorkerFailure("MainBoardState DDS callback");
          }
        },
        1);
  }

  void RecordWorkerFailure(const char* worker) noexcept {
    const std::exception_ptr failure = std::current_exception();
    try {
      std::lock_guard<std::mutex> lock(worker_failure_mutex_);
      if (!worker_failure_) worker_failure_ = failure;
    } catch (...) {
      // Continue into fail-closed shutdown even if diagnostic storage fails.
    }

    shutdown_requested_.store(true, std::memory_order_release);
    try {
      if (safety_) {
        safety_->ForceFault(FaultCode::kExternalFault,
                            std::string(worker) + " threw an exception",
                            Clock::now());
      }
    } catch (...) {
      // The terminal phase and emergency STOP path remain available.
    }
    try {
      BeginControlledShutdown();
    } catch (...) {
    }
  }

  std::exception_ptr CopyWorkerFailure() const {
    std::lock_guard<std::mutex> lock(worker_failure_mutex_);
    return worker_failure_;
  }

  void ControlThreadMain() noexcept {
    try {
      ControlLoop();
    } catch (...) {
      RecordWorkerFailure("control thread");
    }
  }

  void CommandThreadMain() noexcept {
    try {
      CommandLoop();
    } catch (...) {
      RecordWorkerFailure("command thread");
      BestEffortPublishStop();
    }
  }

  void CloseChannels() noexcept {
    try {
      if (mainboard_subscriber_) mainboard_subscriber_->CloseChannel();
      if (low_state_subscriber_) low_state_subscriber_->CloseChannel();
      if (low_command_publisher_) low_command_publisher_->CloseChannel();
    } catch (const std::exception& error) {
      std::cerr << "DDS channel close failed: " << error.what() << std::endl;
    }
    mainboard_subscriber_.reset();
    low_state_subscriber_.reset();
    low_command_publisher_.reset();
  }

  void StopThreadsAndDds() noexcept {
    threads_running_.store(false, std::memory_order_release);
    if (control_thread_.joinable() &&
        control_thread_.get_id() != std::this_thread::get_id()) {
      try {
        control_thread_.join();
      } catch (const std::exception& error) {
        std::cerr << "Control thread join failed: " << error.what()
                  << std::endl;
      }
    }
    if (command_thread_.joinable() &&
        command_thread_.get_id() != std::this_thread::get_id()) {
      try {
        command_thread_.join();
      } catch (const std::exception& error) {
        std::cerr << "Command thread join failed: " << error.what()
                  << std::endl;
      }
    }
    CloseChannels();
    motion_switcher_.reset();
    if (dds_factory_initialized_) {
      try {
        unitree::robot::ChannelFactory::Instance()->Release();
      } catch (const std::exception& error) {
        std::cerr << "DDS factory release failed: " << error.what()
                  << std::endl;
      }
      dds_factory_initialized_ = false;
    }
    if (log_.is_open()) {
      log_.flush();
      log_.close();
    }
  }

  void OnLowState(const void* message) {
    if (message == nullptr || !safety_) return;
    const auto now = Clock::now();
    const LowState state = *static_cast<const LowState*>(message);
    const bool crc_valid = ComputeMessageCrc(state) == state.crc();

    LowStateSnapshot safety_state;
    safety_state.crc_valid = crc_valid;
    safety_state.tick = state.tick();
    safety_state.mode_machine = state.mode_machine();
    if (crc_valid) {
      safety_state.quaternion = state.imu_state().quaternion();
      safety_state.body_angular_velocity = state.imu_state().gyroscope();
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        const std::size_t motor = config_.motor_indices[i];
        safety_state.joint_position[i] = state.motor_state()[motor].q();
        safety_state.joint_velocity[i] = state.motor_state()[motor].dq();
      }
    }
    safety_->ObserveLowState(safety_state, now);
    if (!crc_valid) {
      bad_crc_count_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    RobotSnapshot snapshot;
    snapshot.received = true;
    snapshot.tick = state.tick();
    snapshot.mode_machine = state.mode_machine();
    snapshot.received_at = now;
    snapshot.quaternion = state.imu_state().quaternion();
    snapshot.projected_gravity = ProjectGravity(snapshot.quaternion);
    snapshot.angular_velocity = state.imu_state().gyroscope();
    snapshot.wireless_remote = state.wireless_remote();
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      const std::size_t motor = config_.motor_indices[i];
      snapshot.q[i] = state.motor_state()[motor].q();
      snapshot.dq[i] = state.motor_state()[motor].dq();
    }

    if (!Finite(snapshot.quaternion) || !Finite(snapshot.projected_gravity) ||
        !Finite(snapshot.angular_velocity) || !Finite(snapshot.q) ||
        !Finite(snapshot.dq)) {
      safety_->ForceFault(FaultCode::kNonFiniteState,
                          "LowState contains non-finite policy input", now);
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (robot_state_.received &&
        !TickAdvanced(robot_state_.tick, snapshot.tick)) {
      return;
    }
    snapshot.sequence = robot_state_.sequence + 1;
    robot_state_ = snapshot;
  }

  void OnMainboardState(const void* message) {
    if (message == nullptr || !safety_) return;
    const auto now = Clock::now();
    const MainBoardState state = *static_cast<const MainBoardState*>(message);
    const std::uint32_t state0 = state.state()[0];
    safety_->ObserveMainboard(state0, now);
    std::lock_guard<std::mutex> lock(state_mutex_);
    mainboard_state_.received = true;
    mainboard_state_.state0 = state0;
    mainboard_state_.received_at = now;
    ++mainboard_state_.sequence;
  }

  RobotSnapshot CopyRobotState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return robot_state_;
  }

  MainboardSnapshot CopyMainboardState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return mainboard_state_;
  }

  DesiredCommand CopyDesired() const {
    std::lock_guard<std::mutex> lock(desired_mutex_);
    return desired_;
  }

  void BeginControlledShutdown() {
    std::lock_guard<std::mutex> lock(desired_mutex_);
    if (desired_.phase == RuntimePhase::kStopped ||
        desired_.phase == RuntimePhase::kStopping) {
      return;
    }
    desired_.phase = RuntimePhase::kStopping;
    desired_.shutdown_started = Clock::now();
    desired_.phase_started = desired_.shutdown_started;
    ++desired_.revision;
    std::cout << "Control phase -> STOPPING" << std::endl;
  }

  void MarkStopped() {
    std::lock_guard<std::mutex> lock(desired_mutex_);
    if (desired_.phase == RuntimePhase::kStopped) return;
    desired_.phase = RuntimePhase::kStopped;
    desired_.phase_started = Clock::now();
    ++desired_.revision;
    std::cout << "Control phase -> STOPPED" << std::endl;
  }

  void TransitionTo(RuntimePhase phase, Clock::time_point now,
                    const RobotSnapshot& state) {
    std::lock_guard<std::mutex> lock(desired_mutex_);
    if (desired_.phase == RuntimePhase::kStopping ||
        desired_.phase == RuntimePhase::kStopped) {
      return;
    }
    if (desired_.phase == phase) return;
    desired_.phase = phase;
    desired_.phase_started = now;
    ++desired_.revision;
    if (phase == RuntimePhase::kStand) {
      desired_.stand_start = state.q;
      desired_.requested_q = state.q;
      desired_.sent_q = state.q;
      desired_.stand_complete = false;
    } else if (phase == RuntimePhase::kDamping) {
      desired_.requested_q = state.q;
      desired_.sent_q = state.q;
      desired_.stand_complete = false;
      desired_.velocity_command.fill(0.0F);
    } else if (phase == RuntimePhase::kFault) {
      desired_.velocity_command.fill(0.0F);
    }
    if (phase != RuntimePhase::kControl) {
      filtered_navigation_action_.fill(0.0F);
    }
    std::cout << "Control phase -> " << PhaseName(phase) << std::endl;
  }

  void EnterFaultIfNeeded(const SafetyDecision& decision,
                          Clock::time_point now,
                          const RobotSnapshot& state) {
    if (!decision.fault_latched) return;
    const DesiredCommand desired = CopyDesired();
    if (desired.phase == RuntimePhase::kFault ||
        desired.phase == RuntimePhase::kStopping ||
        desired.phase == RuntimePhase::kStopped) {
      return;
    }
    policy_.Reset();
    TransitionTo(RuntimePhase::kFault, now, state);
    std::cerr << "Safety fault: " << FaultCodeName(decision.fault) << " - "
              << decision.reason << std::endl;
  }

  bool ReadGamepad(const RobotSnapshot& state) {
    static_assert(sizeof(unitree::common::REMOTE_DATA_RX) == 40,
                  "Unitree remote wire payload must remain 40 bytes");
    unitree::common::REMOTE_DATA_RX remote{};
    std::memcpy(remote.buff, state.wireless_remote.data(),
                state.wireless_remote.size());
    const std::array<float, 5> axes{remote.RF_RX.lx, remote.RF_RX.rx,
                                    remote.RF_RX.ry, remote.RF_RX.L2,
                                    remote.RF_RX.ly};
    if (!Finite(axes) ||
        std::any_of(axes.begin(), axes.end(), [](const float value) {
          return std::abs(value) > 1.2F;
        })) {
      safety_->ForceFault(FaultCode::kExternalFault,
                          "wireless remote axis is invalid", Clock::now());
      return false;
    }
    gamepad_.extract(remote);
    return true;
  }

  void ControlLoop() {
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(config_.policy_dt));
    auto next = Clock::now();
    while (threads_running_.load(std::memory_order_acquire)) {
      next += period;
      auto now = Clock::now();
      RobotSnapshot state = CopyRobotState();
      const DesiredCommand phase_snapshot = CopyDesired();
      if ((phase_snapshot.phase == RuntimePhase::kDamping ||
           phase_snapshot.phase == RuntimePhase::kStand ||
           phase_snapshot.phase == RuntimePhase::kControl) &&
          !options_.simulation &&
          now >= next_motion_check_) {
        MotionStillReleased();
        now = Clock::now();
        // The RPC is bounded but may still outlive the 100 ms state watchdog.
        // Never run policy/operator logic on the snapshot taken before it.
        state = CopyRobotState();
      }
      SafetyDecision decision = safety_->Evaluate(now);
      EnterFaultIfNeeded(decision, now, state);

      if (state.received && ReadGamepad(state)) {
        HandleOperatorInput(state, now);
        HandleSubmittedOperatorInput(state, now);
      }

      const MainboardSnapshot mainboard = CopyMainboardState();
      if (!dds_inputs_ready_reported_ && state.received &&
          mainboard.received && mainboard.state0 == 0U &&
          !decision.fault_latched &&
          decision.low_state_age <= config_.low_state_timeout &&
          decision.mainboard_age <= config_.mainboard_runtime_timeout) {
        dds_inputs_ready_reported_ = true;
        std::cout << "A2 DDS inputs healthy; controller is ready to arm"
                  << std::endl;
      }

      const auto update_at = Clock::now();
      decision = safety_->Evaluate(update_at);
      EnterFaultIfNeeded(decision, update_at, state);
      UpdateDesired(state, decision, update_at);
      WriteLog(state, CopyMainboardState(), decision, update_at);

      std::this_thread::sleep_until(next);
      if (Clock::now() - next > period * 4) next = Clock::now();
    }
  }

  void HandleOperatorInput(const RobotSnapshot& state,
                           Clock::time_point now) {
    DesiredCommand desired = CopyDesired();
    if (shutdown_requested_.load(std::memory_order_acquire) ||
        desired.phase == RuntimePhase::kStopping ||
        desired.phase == RuntimePhase::kStopped) {
      return;
    }

    if (desired.phase == RuntimePhase::kFault && gamepad_.LB.pressed &&
        gamepad_.start.on_pressed) {
      if (MotionAuthorityReady()) {
        const auto rearm_at = Clock::now();
        if (!safety_->ExplicitRearm(rearm_at)) {
          std::cerr << "Rearm rejected: safety inputs are not healthy"
                    << std::endl;
          return;
        }
        safety_->RequestMode(ControlMode::kDamping);
        policy_.Reset();
        TransitionTo(RuntimePhase::kDamping, rearm_at, state);
        std::cout << "Fault cleared by explicit L1+START rearm" << std::endl;
      } else {
        std::cerr << "Rearm rejected: motion control is not released"
                  << std::endl;
      }
      return;
    }

    if (desired.phase == RuntimePhase::kPrearm &&
        gamepad_.start.on_pressed) {
      if (!MotionAuthorityReady()) {
        std::cerr << "Arm rejected: motion control is not released"
                  << std::endl;
        return;
      }
      const auto arm_at = Clock::now();
      if (safety_->RequestArm(arm_at)) {
        safety_->RequestMode(ControlMode::kDamping);
        TransitionTo(RuntimePhase::kDamping, arm_at, state);
      }
      return;
    }

    if (gamepad_.LB.pressed && gamepad_.Y.pressed &&
        desired.phase != RuntimePhase::kPrearm &&
        desired.phase != RuntimePhase::kFault &&
        desired.phase != RuntimePhase::kStopping &&
        desired.phase != RuntimePhase::kStopped) {
      safety_->RequestMode(ControlMode::kDamping);
      policy_.Reset();
      TransitionTo(RuntimePhase::kDamping, now, state);
      return;
    }

    if (desired.phase == RuntimePhase::kDamping && gamepad_.LB.pressed &&
        gamepad_.RT.on_pressed && safety_->RequestMode(ControlMode::kStand)) {
      TransitionTo(RuntimePhase::kStand, now, state);
      return;
    }

    if (desired.phase == RuntimePhase::kStand && desired.stand_complete &&
        gamepad_.LB.pressed && gamepad_.A.on_pressed &&
        safety_->RequestMode(ControlMode::kControl)) {
      policy_.Reset();
      TransitionTo(RuntimePhase::kControl, now, state);
    }
  }

  void HandleSubmittedOperatorInput(const RobotSnapshot& state,
                                    Clock::time_point now) {
    std::optional<OperatorRequest> request;
    {
      std::lock_guard<std::mutex> lock(submitted_input_mutex_);
      request = pending_operator_request_;
      pending_operator_request_.reset();
    }
    if (!request.has_value()) return;

    const DesiredCommand desired = CopyDesired();
    if (shutdown_requested_.load(std::memory_order_acquire) ||
        desired.phase == RuntimePhase::kStopping ||
        desired.phase == RuntimePhase::kStopped) {
      return;
    }

    switch (*request) {
      case OperatorRequest::kArmDamping: {
        if (desired.phase != RuntimePhase::kPrearm) {
          std::cerr << "arm rejected: controller is not in PREARM" << std::endl;
          return;
        }
        if (!MotionAuthorityReady()) {
          std::cerr << "arm rejected: motion control is not released"
                    << std::endl;
          return;
        }
        const auto arm_at = Clock::now();
        if (safety_->RequestArm(arm_at)) {
          safety_->RequestMode(ControlMode::kDamping);
          TransitionTo(RuntimePhase::kDamping, arm_at, state);
        } else {
          std::cerr << "arm rejected: safety inputs are not healthy"
                    << std::endl;
        }
        return;
      }
      case OperatorRequest::kStand:
        if (desired.phase == RuntimePhase::kDamping &&
            safety_->RequestMode(ControlMode::kStand)) {
          TransitionTo(RuntimePhase::kStand, now, state);
        } else {
          std::cerr << "stand rejected: arm/mainboard gate is not ready or "
                       "phase is not DAMPING"
                    << std::endl;
        }
        return;
      case OperatorRequest::kControl:
        if (desired.phase == RuntimePhase::kStand &&
            desired.stand_complete &&
            safety_->RequestMode(ControlMode::kControl)) {
          policy_.Reset();
          filtered_navigation_action_.fill(0.0F);
          TransitionTo(RuntimePhase::kControl, now, state);
        } else {
          std::cerr << "ctrl rejected: stand interpolation is not complete"
                    << std::endl;
        }
        return;
      case OperatorRequest::kDamping:
        if (desired.phase != RuntimePhase::kPrearm &&
            desired.phase != RuntimePhase::kFault &&
            safety_->RequestMode(ControlMode::kDamping)) {
          policy_.Reset();
          TransitionTo(RuntimePhase::kDamping, now, state);
        } else {
          std::cerr << "damp rejected: controller is not armed" << std::endl;
        }
        return;
      case OperatorRequest::kRearm: {
        if (desired.phase != RuntimePhase::kFault || !MotionAuthorityReady()) {
          std::cerr << "rearm rejected: controller is not in a recoverable "
                       "FAULT"
                    << std::endl;
          return;
        }
        const auto rearm_at = Clock::now();
        if (!safety_->ExplicitRearm(rearm_at)) {
          std::cerr << "rearm rejected: safety inputs are not healthy"
                    << std::endl;
          return;
        }
        safety_->RequestMode(ControlMode::kDamping);
        policy_.Reset();
        TransitionTo(RuntimePhase::kDamping, rearm_at, state);
        std::cout << "Fault cleared by explicit terminal rearm" << std::endl;
        return;
      }
    }
  }

  Vec3 ResolveNavigationCommand() {
    if (options_.navigation_source == NavigationSource::kGamepad) {
      const Vec3 axes{gamepad_.ly(), -gamepad_.lx(), -gamepad_.rx()};
      return MapCommandAxes(axes, config_.command_lower,
                            config_.command_upper);
    }

    Vec3 raw_action{};
    bool received = false;
    Clock::time_point received_at{};
    {
      std::lock_guard<std::mutex> lock(submitted_input_mutex_);
      raw_action = submitted_navigation_action_;
      received = submitted_navigation_received_;
      received_at = submitted_navigation_at_;
    }

    if (options_.navigation_source == NavigationSource::kHighPolicy) {
      const Clock::time_point freshness_now = Clock::now();
      const bool fresh =
          received && freshness_now >= received_at &&
          freshness_now - received_at <= options_.high_policy_timeout;
      if (!fresh) {
        filtered_navigation_action_.fill(0.0F);
        if (high_policy_connected_) {
          high_policy_connected_ = false;
          std::cerr << "High-policy command timed out; velocity command -> 0"
                    << std::endl;
        }
        return {};
      }
      if (!high_policy_connected_) {
        high_policy_connected_ = true;
        std::cout << "High-policy command stream connected" << std::endl;
      }
    }

    const NavigationFilterResult filtered = FilterNavigationAction(
        raw_action, filtered_navigation_action_, config_.command_lower,
        config_.command_upper);
    filtered_navigation_action_ = filtered.filtered_action;
    return filtered.command;
  }

  void UpdateDesired(const RobotSnapshot& state,
                     const SafetyDecision& decision,
                     Clock::time_point now) {
    if (!state.received) {
      policy_deadline_watchdog_.Reset();
      return;
    }

    std::uint64_t inference_revision = 0;
    {
      std::lock_guard<std::mutex> lock(desired_mutex_);
      if (desired_.phase == RuntimePhase::kDamping) {
        policy_deadline_watchdog_.Reset();
        desired_.requested_q = state.q;
        desired_.velocity_command.fill(0.0F);
        return;
      }

      if (desired_.phase == RuntimePhase::kStand) {
        policy_deadline_watchdog_.Reset();
        const double elapsed = std::chrono::duration<double>(
                                   now - desired_.phase_started)
                                   .count();
        const double duration =
            std::chrono::duration<double>(config_.stand_duration).count();
        const float ratio = static_cast<float>(
            std::clamp(elapsed / duration, 0.0, 1.0));
        for (std::size_t i = 0; i < kNumJoints; ++i) {
          desired_.requested_q[i] =
              desired_.stand_start[i] +
              ratio * (config_.default_q[i] - desired_.stand_start[i]);
        }
        desired_.stand_complete = ratio >= 1.0F;
        desired_.velocity_command.fill(0.0F);
        return;
      }

      if (desired_.phase != RuntimePhase::kControl ||
          !decision.policy_allowed ||
          shutdown_requested_.load(std::memory_order_acquire)) {
        policy_deadline_watchdog_.Reset();
        return;
      }
      inference_revision = desired_.revision;
    }

    try {
      const Vec3 velocity_command = ResolveNavigationCommand();
      const auto inference_started = Clock::now();
      const PolicyResult policy_result = policy_.Infer(
          velocity_command, state.projected_gravity, state.angular_velocity,
          state.q, state.dq);
      const auto inference_finished = Clock::now();
      const double policy_time_ms =
          std::chrono::duration<double, std::milli>(inference_finished -
                                                    inference_started)
              .count();
      const double policy_deadline_ms = config_.policy_dt * 1000.0;
      const PolicyDeadlineDecision deadline_decision =
          policy_deadline_watchdog_.Observe(policy_time_ms,
                                             policy_deadline_ms);
      if (deadline_decision.fatal) {
        std::ostringstream reason;
        reason << "policy inference deadline failure: " << policy_time_ms
               << " ms (soft=" << policy_deadline_ms
               << " ms, hard="
               << policy_deadline_ms * kPolicyHardDeadlineMultiplier
               << " ms, consecutive_misses="
               << deadline_decision.consecutive_misses << ')';
        safety_->ForceFault(FaultCode::kExternalFault, reason.str(),
                            inference_finished);
        policy_.Reset();
        policy_deadline_watchdog_.Reset();
        return;
      }
      if (deadline_decision.missed) {
        std::cerr << "Policy soft deadline miss "
                  << deadline_decision.consecutive_misses << '/'
                  << kPolicyDeadlineMissLimit << ": " << policy_time_ms
                  << " ms > " << policy_deadline_ms
                  << " ms; fresh result remains eligible" << std::endl;
      }

      const SafetyDecision commit_decision =
          safety_->Evaluate(inference_finished);
      const bool inference_state_fresh =
          inference_finished - state.received_at <= config_.low_state_timeout;
      bool committed = false;
      {
        std::lock_guard<std::mutex> lock(desired_mutex_);
        if (desired_.phase == RuntimePhase::kControl &&
            desired_.revision == inference_revision &&
            commit_decision.policy_allowed && inference_state_fresh &&
            !shutdown_requested_.load(std::memory_order_acquire)) {
          desired_.velocity_command = velocity_command;
          desired_.policy_result = policy_result;
          desired_.policy_time_ms = policy_time_ms;
          desired_.requested_q = policy_result.target_q;
          committed = true;
        }
      }
      if (!committed) policy_.Reset();
    } catch (const std::exception& error) {
      safety_->ForceFault(FaultCode::kExternalFault,
                          std::string("policy failure: ") + error.what(), now);
      policy_.Reset();
    }
  }

  void CommandLoop() {
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(config_.command_dt));
    auto next = Clock::now();
    while (threads_running_.load(std::memory_order_acquire)) {
      next += period;
      BuildAndPublishCommand(Clock::now());
      std::this_thread::sleep_until(next);
      if (Clock::now() - next > period * 10) next = Clock::now();
    }
  }

  void BestEffortPublishStop() noexcept {
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(config_.command_dt));
    const auto deadline = Clock::now() + config_.stop_duration;
    auto next = Clock::now();
    do {
      next += period;
      try {
        const RobotSnapshot state = CopyRobotState();
        if (state.received &&
            state.mode_machine == config_.expected_mode_machine &&
            low_command_publisher_) {
          LowCommandBuildInput input;
          input.kind = LowCommandKind::kStop;
          input.mode_machine = state.mode_machine;
          const LowCmd command = BuildLowCommand(input);
          low_command_publisher_->Write(command);
        }
      } catch (...) {
        // Keep retrying STOP until the bounded emergency window expires.
      }
      std::this_thread::sleep_until(next);
    } while (Clock::now() < deadline);
    have_previous_sent_target_ = false;
  }

  void BuildAndPublishCommand(Clock::time_point now) {
    const RobotSnapshot state = CopyRobotState();
    if (!state.received || state.mode_machine != config_.expected_mode_machine ||
        !low_command_publisher_) {
      return;
    }

    const DesiredCommand desired = CopyDesired();
    const SafetyDecision decision = safety_->Evaluate(now);
    RuntimePhase effective_phase = desired.phase;
    if (decision.fault_latched && effective_phase != RuntimePhase::kFault &&
        effective_phase != RuntimePhase::kStopping) {
      effective_phase = RuntimePhase::kFault;
    }

    bool use_foc = false;
    bool damping = false;
    const bool recent_state =
        now - state.received_at <= config_.low_state_timeout;
    if (effective_phase == RuntimePhase::kDamping ||
        effective_phase == RuntimePhase::kStand ||
        effective_phase == RuntimePhase::kControl) {
      use_foc = decision.foc_allowed && recent_state;
      damping = effective_phase == RuntimePhase::kDamping;
    } else if (effective_phase == RuntimePhase::kFault) {
      use_foc = FaultDampingAuthorized(decision, recent_state,
                                       foc_has_been_published_);
      damping = use_foc;
    } else if (effective_phase == RuntimePhase::kStopping) {
      const bool within_damping =
          desired.shutdown_started.time_since_epoch().count() != 0 &&
          now - desired.shutdown_started < config_.damping_duration;
      use_foc =
          within_damping && ShutdownDampingAuthorized(
                                decision, recent_state,
                                foc_has_been_published_);
      damping = use_foc;
    }

    LowCommandBuildInput command_input;
    command_input.mode_machine = state.mode_machine;
    command_input.motor_indices = config_.motor_indices;
    command_input.joint_position = state.q;
    command_input.joint_velocity = state.dq;
    command_input.kp = config_.kp;
    command_input.kd = config_.kd;
    command_input.effort_limits = config_.effort_limits;

    std::size_t limiter_count = 0;
    JointArray sent_target = state.q;
    if (use_foc) {
      if (damping) {
        command_input.kind = LowCommandKind::kDamping;
        sent_target = state.q;
        have_previous_sent_target_ = false;
      } else {
        if (!have_previous_sent_target_) {
          previous_sent_target_ = state.q;
          have_previous_sent_target_ = true;
        }
        TargetFilterInput filter_input;
        filter_input.desired_target = desired.requested_q;
        filter_input.previous_target = previous_sent_target_;
        filter_input.joint_position = state.q;
        filter_input.joint_velocity = state.dq;
        const bool standing = effective_phase == RuntimePhase::kStand;
        const auto filtered = FilterTarget(
            filter_input, standing ? config_.hard_lower : config_.soft_lower,
            standing ? config_.hard_upper : config_.soft_upper,
            config_.target_velocity_limits, config_.effort_limits, config_.kp,
            config_.kd, config_.command_dt);
        sent_target = filtered.target;
        previous_sent_target_ = sent_target;
        for (std::size_t i = 0; i < kNumJoints; ++i) {
          if (filtered.position_limited[i] || filtered.velocity_limited[i] ||
              filtered.effort_limited[i]) {
            ++limiter_count;
          }
        }
        command_input.kind = LowCommandKind::kPosition;
        command_input.target_position = sent_target;
      }
    } else {
      have_previous_sent_target_ = false;
      command_input.kind = LowCommandKind::kStop;
    }

    // Recheck immediately before serialization/publication. This check may
    // only retain or reduce the authority selected above; it can never turn a
    // STOP decision into FOC within the same command iteration.
    if (use_foc) {
      const auto publish_at = Clock::now();
      const bool publish_state_fresh =
          publish_at - state.received_at <= config_.low_state_timeout;
      const SafetyDecision publish_decision = safety_->Evaluate(publish_at);
      bool publish_foc = false;
      bool publish_damping = false;
      if (publish_decision.fault_latched) {
        publish_foc = FaultDampingAuthorized(
            publish_decision, publish_state_fresh, foc_has_been_published_);
        publish_damping = publish_foc;
      } else if (effective_phase == RuntimePhase::kStopping) {
        const bool within_damping =
            desired.shutdown_started.time_since_epoch().count() != 0 &&
            publish_at - desired.shutdown_started < config_.damping_duration;
        publish_foc =
            within_damping && ShutdownDampingAuthorized(
                                  publish_decision, publish_state_fresh,
                                  foc_has_been_published_);
        publish_damping = publish_foc;
      } else if (effective_phase == RuntimePhase::kDamping ||
                 effective_phase == RuntimePhase::kStand ||
                 effective_phase == RuntimePhase::kControl) {
        publish_foc = publish_state_fresh && publish_decision.foc_allowed &&
                      publish_decision.command_authority ==
                          CommandAuthority::kActive;
        publish_damping =
            publish_foc && effective_phase == RuntimePhase::kDamping;
      }
      use_foc = publish_foc;
      damping = publish_damping;
      if (!use_foc) {
        command_input.kind = LowCommandKind::kStop;
        have_previous_sent_target_ = false;
        sent_target = state.q;
      } else if (damping) {
        command_input.kind = LowCommandKind::kDamping;
        have_previous_sent_target_ = false;
        sent_target = state.q;
      }
    }

    const LowCmd command = BuildLowCommand(command_input);
    const bool write_succeeded = low_command_publisher_->Write(command);
    if (write_succeeded && use_foc) foc_has_been_published_ = true;
    if (!write_succeeded) {
      safety_->ForceFault(FaultCode::kExternalFault,
                          "LowCmd DDS write failed", now);
    }

    {
      std::lock_guard<std::mutex> lock(desired_mutex_);
      desired_.limiter_count = limiter_count;
      // Telemetry may observe the transmitted target, but the 500 Hz command
      // thread must never feed it back into the 50 Hz requested target.
      if (write_succeeded && use_foc) desired_.sent_q = sent_target;
    }
  }

  void WriteLog(const RobotSnapshot& state,
                const MainboardSnapshot& mainboard,
                const SafetyDecision& decision,
                Clock::time_point now) {
    if (!log_.is_open()) return;
    const DesiredCommand desired = CopyDesired();
    const auto elapsed =
        std::chrono::duration<double, std::milli>(now - runtime_started_)
            .count();
    log_ << std::fixed << std::setprecision(6) << elapsed << ','
         << PhaseName(desired.phase) << ',' << FaultCodeName(decision.fault)
         << ',' << CsvSafe(decision.reason) << ','
         << decision.low_state_age.count() << ','
         << decision.mainboard_age.count() << ',' << mainboard.state0 << ',';
    for (const float value : desired.velocity_command) log_ << value << ',';
    for (const float value : state.projected_gravity) log_ << value << ',';
    for (const float value : state.angular_velocity) log_ << value << ',';
    for (const float value : state.q) log_ << value << ',';
    for (const float value : state.dq) log_ << value << ',';
    for (const float value : desired.policy_result.action) log_ << value << ',';
    for (const float value : desired.requested_q) log_ << value << ',';
    for (const float value : desired.sent_q) log_ << value << ',';
    log_ << desired.limiter_count << ',' << desired.policy_time_ms << '\n';
    if (++log_rows_since_flush_ >= 50) {
      log_.flush();
      log_rows_since_flush_ = 0;
    }
  }

  A2Config config_;
  RuntimeOptions options_;
  A2Policy policy_;
  std::unique_ptr<SafetySupervisor> safety_;

  std::unique_ptr<unitree::robot::b2::MotionSwitcherClient> motion_switcher_;
  std::string last_motion_form_;
  std::string last_motion_mode_;

  unitree::robot::ChannelPublisherPtr<LowCmd> low_command_publisher_;
  unitree::robot::ChannelSubscriberPtr<LowState> low_state_subscriber_;
  unitree::robot::ChannelSubscriberPtr<MainBoardState> mainboard_subscriber_;

  mutable std::mutex state_mutex_;
  RobotSnapshot robot_state_;
  MainboardSnapshot mainboard_state_;

  mutable std::mutex desired_mutex_;
  DesiredCommand desired_;
  JointArray previous_sent_target_{};
  bool have_previous_sent_target_{false};
  bool foc_has_been_published_{false};

  mutable std::mutex submitted_input_mutex_;
  std::optional<OperatorRequest> pending_operator_request_;
  Vec3 submitted_navigation_action_{};
  Clock::time_point submitted_navigation_at_{};
  bool submitted_navigation_received_{false};
  Vec3 filtered_navigation_action_{};
  bool high_policy_connected_{false};
  bool dds_inputs_ready_reported_{false};
  PolicyDeadlineWatchdog policy_deadline_watchdog_;

  unitree::common::UnitreeJoystick gamepad_;
  std::atomic<bool> shutdown_requested_{false};
  std::atomic<bool> threads_running_{false};
  std::atomic<std::uint64_t> bad_crc_count_{0};
  bool dds_factory_initialized_{false};
  std::thread command_thread_;
  std::thread control_thread_;
  Clock::time_point next_motion_check_{Clock::now()};

  mutable std::mutex worker_failure_mutex_;
  std::exception_ptr worker_failure_;

  std::ofstream log_;
  Clock::time_point runtime_started_{Clock::now()};
  std::size_t log_rows_since_flush_{0};
};

A2Controller::A2Controller(A2Config config, RuntimeOptions options)
    : impl_(std::make_unique<Impl>(std::move(config), std::move(options))) {}

A2Controller::~A2Controller() = default;

int A2Controller::Run() { return impl_->Run(); }

void A2Controller::RequestShutdown() { impl_->RequestShutdown(); }

void A2Controller::SubmitOperatorRequest(const OperatorRequest request) {
  impl_->SubmitOperatorRequest(request);
}

void A2Controller::SubmitNavigationAction(const Vec3& action) {
  impl_->SubmitNavigationAction(action);
}

}  // namespace a2

#pragma once

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>

#include "a2/config.hpp"
#include "a2/contract.hpp"

namespace a2 {

enum class NavigationSource {
  kGamepad = 0,
  kTerminal = 1,
  kHighPolicy = 2,
};

enum class OperatorRequest {
  kArmDamping = 0,
  kStand = 1,
  kControl = 2,
  kDamping = 3,
  kRearm = 4,
};

struct RuntimeOptions {
  std::string network_interface;
  std::filesystem::path log_file;
  int domain_id{0};
  bool simulation{false};
  NavigationSource navigation_source{NavigationSource::kGamepad};
  std::chrono::milliseconds high_policy_timeout{200};
};

class A2Controller {
 public:
  A2Controller(A2Config config, RuntimeOptions options);
  ~A2Controller();

  A2Controller(const A2Controller&) = delete;
  A2Controller& operator=(const A2Controller&) = delete;

  int Run();
  void RequestShutdown();

  // Simulation/operator mode requests remain separate from navigation. A
  // future high-level policy only submits [vx, vy, yaw_rate]; it never gets
  // direct access to the arm/mode safety state machine or motor commands.
  void SubmitOperatorRequest(OperatorRequest request);
  void SubmitNavigationAction(const Vec3& action);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace a2

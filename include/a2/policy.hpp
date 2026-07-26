#pragma once

#include <array>
#include <string>

#include <torch/script.h>

#include "a2/config.hpp"

namespace a2 {

struct PolicyResult {
  std::array<float, 45> observation{};
  std::array<float, 12> action{};
  std::array<float, 12> target_q{};
};

class A2Policy {
 public:
  explicit A2Policy(const A2Config& config);

  void Load();
  void Reset();

  PolicyResult Infer(const std::array<float, 3>& command,
                     const std::array<float, 3>& projected_gravity,
                     const std::array<float, 3>& body_angular_velocity,
                     const std::array<float, 12>& joint_position,
                     const std::array<float, 12>& joint_velocity);

  const std::array<float, 12>& previous_action() const {
    return previous_action_;
  }

  static std::string Sha256File(const std::string& path);

 private:
  std::array<float, 12> Forward(const std::array<float, 45>& observation);
  void VerifyProbeOutput();

  const A2Config& config_;
  torch::jit::script::Module module_;
  std::array<float, 12> previous_action_{};
  bool loaded_{false};
};

}  // namespace a2

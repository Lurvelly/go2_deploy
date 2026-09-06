#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#include "a2/config.hpp"
#include "a2/contract.hpp"
#include "a2/policy.hpp"

namespace {

int failures = 0;

void Check(const bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

template <typename Callable>
void ExpectThrow(Callable&& callable, const std::string& message) {
  try {
    callable();
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

bool Near(const float lhs, const float rhs,
          const float tolerance = 5.0e-5F) {
  return std::fabs(lhs - rhs) <= tolerance;
}

void TestPolicyArtifactAndGoldenInference() {
  const a2::A2Config config =
      a2::A2Config::Load(RepositoryRoot() / "params" / "a2.yaml");
  Check(a2::A2Policy::Sha256File(config.model_path.string()) ==
            std::string(a2::kBundledPolicySha256),
        "bundled policy bytes match the pinned SHA-256 identity");

  a2::A2Policy policy(config);
  policy.Load();
  for (const float action : policy.previous_action()) {
    Check(action == 0.0F, "Load resets the recurrent previous-action input");
  }

  constexpr std::array<float, a2::kActionDim> expected{
      -1.75422192F, -0.643863201F, 1.05439806F,  -0.525758445F,
      -1.86393595F, 0.417357594F,  -2.31677747F, 2.03326392F,
      2.73407602F,  -0.247133166F, 3.92397285F,  2.11677814F,
  };

  const std::array<float, 3> command{};
  const std::array<float, 3> gravity{0.0F, 0.0F, -1.0F};
  const std::array<float, 3> angular_velocity{};
  a2::JointArray velocity{};
  const a2::PolicyResult first =
      policy.Infer(command, gravity, angular_velocity, config.default_q,
                   velocity);

  Check(first.observation.size() == a2::kObservationDim,
        "policy observation remains exactly 45D");
  Check(first.action.size() == a2::kActionDim,
        "policy action remains exactly 12D");
  Check(first.observation[5] == -1.0F,
        "golden probe places upright gravity at observation index 5");
  for (std::size_t i = 0; i < expected.size(); ++i) {
    Check(std::isfinite(first.action[i]) && Near(first.action[i], expected[i]),
          "golden policy output matches at action index " +
              std::to_string(i));
    Check(Near(first.target_q[i],
               config.default_q[i] + config.action_scale * expected[i]),
          "policy target applies q0 + action_scale * action at index " +
              std::to_string(i));
    Check(Near(policy.previous_action()[i], first.action[i]),
          "raw action is retained for the next observation at index " +
              std::to_string(i));
  }

  const a2::PolicyResult second =
      policy.Infer(command, gravity, angular_velocity, config.default_q,
                   velocity);
  for (std::size_t i = 0; i < a2::kActionDim; ++i) {
    Check(Near(second.observation[33 + i], first.action[i]),
          "second observation contains previous raw action at index " +
              std::to_string(i));
    Check(std::isfinite(second.action[i]) && std::isfinite(second.target_q[i]),
          "recurrent policy inference remains finite at index " +
              std::to_string(i));
  }

  policy.Reset();
  const a2::PolicyResult reset =
      policy.Infer(command, gravity, angular_velocity, config.default_q,
                   velocity);
  for (std::size_t i = 0; i < a2::kActionDim; ++i) {
    Check(reset.observation[33 + i] == 0.0F,
          "Reset clears the previous-action observation at index " +
              std::to_string(i));
    Check(Near(reset.action[i], first.action[i]),
          "Reset reproduces deterministic golden action at index " +
              std::to_string(i));
  }

  a2::A2Config custom_weights = config;
  custom_weights.policy_golden_output.reset();
  a2::A2Policy custom_policy(custom_weights);
  try {
    custom_policy.Load();
  } catch (const std::exception&) {
    Check(false, "a custom-weight config can use the finite shape probe");
  }

  a2::A2Config wrong_reference = config;
  (*wrong_reference.policy_golden_output)[0] += 1.0F;
  ExpectThrow(
      [&] {
        a2::A2Policy rejected(wrong_reference);
        rejected.Load();
      },
      "a configured model-specific reference output is enforced");
}

void TestPolicyLoadFailures() {
  ExpectThrow(
      [] {
        (void)a2::A2Policy::Sha256File(
            (RepositoryRoot() / "models" / "missing-a2-policy.jit").string());
      },
      "missing policy artifact is rejected");
}

}  // namespace

int main() {
  TestPolicyArtifactAndGoldenInference();
  TestPolicyLoadFailures();
  if (failures != 0) {
    std::cerr << failures << " policy test(s) failed\n";
    return 1;
  }
  std::cout << "A2 policy tests passed\n";
  return 0;
}

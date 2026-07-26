#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace a2 {

inline constexpr std::size_t kPolicyDeadlineMissLimit = 3;
inline constexpr double kPolicyHardDeadlineMultiplier = 2.0;

struct PolicyDeadlineDecision {
  bool missed{false};
  bool fatal{false};
  bool accept_result{true};
  std::size_t consecutive_misses{0};
};

// A single inference can be delayed slightly by the host scheduler. Treat the
// policy period as a soft deadline, while retaining bounded fail-closed
// behavior for sustained misses or one inference that consumes two periods.
class PolicyDeadlineWatchdog {
 public:
  PolicyDeadlineDecision Observe(const double elapsed_ms,
                                 const double deadline_ms) {
    if (!std::isfinite(elapsed_ms) || elapsed_ms < 0.0 ||
        !std::isfinite(deadline_ms) || deadline_ms <= 0.0) {
      throw std::invalid_argument("invalid policy deadline observation");
    }

    if (elapsed_ms <= deadline_ms) {
      Reset();
      return {};
    }

    if (consecutive_misses_ < kPolicyDeadlineMissLimit) {
      ++consecutive_misses_;
    }
    const bool hard_miss =
        elapsed_ms >= deadline_ms * kPolicyHardDeadlineMultiplier;
    const bool fatal =
        hard_miss || consecutive_misses_ >= kPolicyDeadlineMissLimit;
    return PolicyDeadlineDecision{true, fatal, !fatal, consecutive_misses_};
  }

  void Reset() noexcept { consecutive_misses_ = 0; }

  std::size_t consecutive_misses() const noexcept {
    return consecutive_misses_;
  }

 private:
  std::size_t consecutive_misses_{0};
};

}  // namespace a2

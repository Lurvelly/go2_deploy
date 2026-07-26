#include <cmath>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "a2/high_command_receiver.hpp"
#include "a2/navigation.hpp"

namespace {

int failures = 0;

void Check(const bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

bool Near(const float lhs, const float rhs) {
  return std::fabs(lhs - rhs) <= 1.0e-6F;
}

void TestDrainBudget() {
  constexpr std::size_t kBudget = 4;
  std::vector<std::string> payloads;
  for (std::size_t i = 0; i < 10; ++i) {
    payloads.push_back(a2::EncodeNavigationDatagram(
        {static_cast<float>(i), 0.0F, 0.0F}));
  }
  std::size_t next_payload = 0;
  const a2::HighCommandDatagramReader receive =
      [&]() -> std::optional<std::string> {
    if (next_payload == payloads.size()) return std::nullopt;
    return payloads[next_payload++];
  };
  bool discarding_backlog = false;

  const a2::HighCommandPollResult first =
      a2::DrainHighCommandDatagrams(receive, kBudget, discarding_backlog);
  Check(first.datagrams_received == kBudget && first.drain_limit_reached,
        "one poll stops at its datagram budget");
  Check(!first.latest_action.has_value() && discarding_backlog,
        "a saturated poll suppresses potentially stale actions");

  const a2::HighCommandPollResult second =
      a2::DrainHighCommandDatagrams(receive, kBudget, discarding_backlog);
  Check(second.datagrams_received == kBudget && second.drain_limit_reached,
        "a later poll continues draining without starving its caller");
  Check(!second.latest_action.has_value() && discarding_backlog,
        "queued actions remain suppressed while backlog draining continues");

  const a2::HighCommandPollResult third =
      a2::DrainHighCommandDatagrams(receive, kBudget, discarding_backlog);
  Check(third.datagrams_received == 2 && !third.drain_limit_reached,
        "the final poll reports when the socket was fully drained");
  Check(!third.latest_action.has_value() && !discarding_backlog,
        "the final stale backlog batch is discarded before rearming input");

  const a2::HighCommandPollResult empty =
      a2::DrainHighCommandDatagrams(receive, kBudget, discarding_backlog);
  Check(empty.datagrams_received == 0 && !empty.latest_action.has_value(),
        "an empty nonblocking poll returns immediately");

  payloads.push_back(a2::EncodeNavigationDatagram({0.25F, 0.0F, 0.0F}));
  const a2::HighCommandPollResult fresh =
      a2::DrainHighCommandDatagrams(receive, kBudget, discarding_backlog);
  Check(fresh.latest_action.has_value() &&
            Near((*fresh.latest_action)[0], 0.25F),
        "the first packet after a fully drained backlog is accepted");
}

}  // namespace

int main() {
  TestDrainBudget();
  if (failures != 0) {
    std::cerr << failures << " high-command receiver test(s) failed\n";
    return 1;
  }
  std::cout << "A2 high-command receiver tests passed\n";
  return 0;
}

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <string>

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

void ExpectThrow(const std::function<void()>& callback,
                 const std::string& message) {
  try {
    callback();
    Check(false, message + " (no exception)");
  } catch (const std::exception&) {
  }
}

void TestSeaNavFilterContract() {
  const a2::NavigationFilterResult first =
      a2::FilterNavigationAction({1.0F, -1.0F, 0.5F}, {});
  Check(Near(first.filtered_action[0], 0.5F) &&
            Near(first.filtered_action[1], -0.5F) &&
            Near(first.filtered_action[2], 0.25F) &&
            Near(first.command[0], 0.5F) && Near(first.command[1], -0.5F) &&
            Near(first.command[2], 0.25F),
        "first action uses alpha=0.5 and the A2 physical envelope");

  const a2::NavigationFilterResult second = a2::FilterNavigationAction(
      {3.0F, 3.0F, -3.0F}, first.filtered_action);
  Check(Near(second.filtered_action[0], 1.75F) &&
            Near(second.filtered_action[1], 1.25F) &&
            Near(second.filtered_action[2], -1.375F) &&
            Near(second.command[0], 1.0F) && Near(second.command[1], 0.5F) &&
            Near(second.command[2], -1.0F),
        "EMA result is clipped to the frozen A2 command bounds");

  const a2::NavigationFilterResult recovery = a2::FilterNavigationAction(
      {}, second.filtered_action);
  Check(Near(recovery.filtered_action[0], 0.875F) &&
            Near(recovery.command[0], 0.875F),
        "the next EMA step uses the unclipped filtered action");

  const a2::NavigationFilterResult raw_clamped =
      a2::FilterNavigationAction({100.0F, 0.0F, 0.0F}, {});
  Check(Near(raw_clamped.filtered_action[0], 1.5F) &&
            Near(raw_clamped.command[0], 1.0F),
        "raw high-level actions are first clipped to [-3,3]");
}

void TestDatagramContract() {
  const a2::Vec3 action = a2::ParseNavigationDatagram("A2NAV1 0.3 -0.2 0.5");
  Check(Near(action[0], 0.3F) && Near(action[1], -0.2F) &&
            Near(action[2], 0.5F),
        "versioned high-policy datagram parses three actions");

  const std::string encoded = a2::EncodeNavigationDatagram(action);
  const a2::Vec3 round_trip = a2::ParseNavigationDatagram(encoded);
  Check(Near(round_trip[0], action[0]) && Near(round_trip[1], action[1]) &&
            Near(round_trip[2], action[2]),
        "high-policy datagram round-trips");

  ExpectThrow([] { (void)a2::ParseNavigationDatagram("0.3 -0.2 0.5"); },
              "unversioned datagrams are rejected");
  ExpectThrow([] { (void)a2::ParseNavigationDatagram("A2NAV2 0 0 0"); },
              "unknown protocol versions are rejected");
  ExpectThrow([] { (void)a2::ParseNavigationDatagram("A2NAV1 0 0 nan"); },
              "non-finite datagrams are rejected");
  ExpectThrow(
      [] {
        (void)a2::EncodeNavigationDatagram(
            {0.0F, std::numeric_limits<float>::infinity(), 0.0F});
      },
      "non-finite actions cannot be encoded");
}

}  // namespace

int main() {
  TestSeaNavFilterContract();
  TestDatagramContract();
  if (failures != 0) {
    std::cerr << failures << " navigation test(s) failed\n";
    return 1;
  }
  std::cout << "A2 navigation tests passed\n";
  return 0;
}

#pragma once

#include <string>
#include <string_view>

#include "a2/contract.hpp"

namespace a2 {

inline constexpr float kNavigationFilterAlpha = 0.5F;
inline constexpr float kNavigationRawLimit = 3.0F;
inline constexpr std::string_view kNavigationDatagramVersion = "A2NAV1";

struct NavigationFilterResult {
  Vec3 filtered_action{};
  Vec3 command{};
};

// Reproduces the Legged-Nexus SEA-NAV high/low boundary: clamp the raw
// navigation action to [-3, 3], apply a one-pole EMA, then clip the physical
// velocity command to the low-policy envelope.
NavigationFilterResult FilterNavigationAction(
    const Vec3& raw_action, const Vec3& previous_filtered,
    const Vec3& command_lower = kCommandLower,
    const Vec3& command_upper = kCommandUpper,
    float alpha = kNavigationFilterAlpha);

// Language-neutral loopback transport for a future high-policy process.
// Wire format: "A2NAV1 <vx> <vy> <yaw_rate>".
Vec3 ParseNavigationDatagram(std::string_view payload);
std::string EncodeNavigationDatagram(const Vec3& action);

}  // namespace a2

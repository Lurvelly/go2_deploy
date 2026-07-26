#include "a2/navigation.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace a2 {

NavigationFilterResult FilterNavigationAction(
    const Vec3& raw_action, const Vec3& previous_filtered,
    const Vec3& command_lower, const Vec3& command_upper, const float alpha) {
  if (!AllFinite(raw_action) || !AllFinite(previous_filtered) ||
      !AllFinite(command_lower) || !AllFinite(command_upper) ||
      !std::isfinite(alpha) || alpha <= 0.0F || alpha > 1.0F) {
    throw std::invalid_argument("invalid navigation filter input");
  }

  NavigationFilterResult result;
  for (std::size_t i = 0; i < result.filtered_action.size(); ++i) {
    if (command_lower[i] > command_upper[i]) {
      throw std::invalid_argument("navigation command bounds are inverted");
    }
    const float raw =
        std::clamp(raw_action[i], -kNavigationRawLimit, kNavigationRawLimit);
    result.filtered_action[i] =
        alpha * raw + (1.0F - alpha) * previous_filtered[i];
    result.command[i] = std::clamp(result.filtered_action[i], command_lower[i],
                                   command_upper[i]);
  }
  return result;
}

Vec3 ParseNavigationDatagram(const std::string_view payload) {
  std::istringstream input{std::string(payload)};
  std::string version;
  Vec3 action{};
  std::string trailing;
  if (!(input >> version >> action[0] >> action[1] >> action[2]) ||
      version != kNavigationDatagramVersion || (input >> trailing) ||
      !AllFinite(action)) {
    throw std::invalid_argument(
        "high-policy datagram must be: A2NAV1 <vx> <vy> <yaw_rate>");
  }
  return action;
}

std::string EncodeNavigationDatagram(const Vec3& action) {
  if (!AllFinite(action)) {
    throw std::invalid_argument("high-policy action contains NaN or Inf");
  }
  std::ostringstream output;
  output << kNavigationDatagramVersion << ' ' << std::setprecision(9)
         << action[0] << ' ' << action[1] << ' ' << action[2];
  return output.str();
}

}  // namespace a2

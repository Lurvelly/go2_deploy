#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "a2/contract.hpp"

namespace a2 {

inline constexpr std::size_t kHighCommandMaxDatagramsPerPoll = 64;
inline constexpr std::size_t kHighCommandDatagramBufferSize = 256;

struct HighCommandPollResult {
  std::optional<Vec3> latest_action;
  std::size_t datagrams_received{0};
  bool drain_limit_reached{false};
};

using HighCommandDatagramReader =
    std::function<std::optional<std::string>()>;

// discarding_backlog is persistent fail-closed state. When true, queued
// datagrams are discarded until the reader reports that its queue is empty.
HighCommandPollResult DrainHighCommandDatagrams(
    const HighCommandDatagramReader& receive,
    std::size_t max_datagrams_per_poll,
    bool& discarding_backlog);

class HighCommandReceiver {
 public:
  explicit HighCommandReceiver(
      std::uint16_t port,
      std::size_t max_datagrams_per_poll =
          kHighCommandMaxDatagramsPerPoll);
  ~HighCommandReceiver();

  HighCommandReceiver(const HighCommandReceiver&) = delete;
  HighCommandReceiver& operator=(const HighCommandReceiver&) = delete;

  HighCommandPollResult ReceiveLatest() noexcept;
  std::uint16_t port() const noexcept { return port_; }

 private:
  int socket_{-1};
  std::uint16_t port_{0};
  std::size_t max_datagrams_per_poll_{0};
  bool discarding_backlog_{false};
};

}  // namespace a2

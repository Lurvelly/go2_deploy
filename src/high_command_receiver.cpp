#include "a2/high_command_receiver.hpp"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "a2/navigation.hpp"

namespace a2 {

HighCommandReceiver::HighCommandReceiver(
    const std::uint16_t port, const std::size_t max_datagrams_per_poll)
    : max_datagrams_per_poll_(max_datagrams_per_poll) {
  if (max_datagrams_per_poll_ == 0) {
    throw std::invalid_argument(
        "high-command datagram budget must be positive");
  }

  socket_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (socket_ < 0) {
    throw std::runtime_error(
        std::string("cannot create high-command UDP socket: ") +
        std::strerror(errno));
  }

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = htons(port);
  if (::bind(socket_, reinterpret_cast<const sockaddr*>(&address),
             sizeof(address)) != 0) {
    const std::string reason = std::strerror(errno);
    ::close(socket_);
    socket_ = -1;
    throw std::runtime_error("cannot bind high-command UDP socket: " + reason);
  }

  socklen_t address_size = sizeof(address);
  if (::getsockname(socket_, reinterpret_cast<sockaddr*>(&address),
                    &address_size) != 0) {
    const std::string reason = std::strerror(errno);
    ::close(socket_);
    socket_ = -1;
    throw std::runtime_error("cannot inspect high-command UDP socket: " +
                             reason);
  }
  port_ = ntohs(address.sin_port);
}

HighCommandReceiver::~HighCommandReceiver() {
  if (socket_ >= 0) ::close(socket_);
}

HighCommandPollResult DrainHighCommandDatagrams(
    const HighCommandDatagramReader& receive,
    const std::size_t max_datagrams_per_poll,
    bool& discarding_backlog) {
  if (max_datagrams_per_poll == 0) {
    throw std::invalid_argument(
        "high-command datagram budget must be positive");
  }

  HighCommandPollResult result;
  while (result.datagrams_received < max_datagrams_per_poll) {
    const std::optional<std::string> payload = receive();
    if (!payload.has_value()) {
      discarding_backlog = false;
      return result;
    }
    ++result.datagrams_received;

    // Once one poll exhausts its budget, every queued packet is potentially
    // stale. Drain to EAGAIN across later polls and wait for a subsequent packet
    // instead of refreshing the watchdog with backlog data.
    if (discarding_backlog) continue;

    if (payload->empty() ||
        payload->size() >= kHighCommandDatagramBufferSize) {
      std::cerr << "ignored empty or oversized high-policy datagram"
                << std::endl;
      continue;
    }
    try {
      result.latest_action = ParseNavigationDatagram(*payload);
    } catch (const std::exception& error) {
      std::cerr << "ignored invalid high-policy datagram: " << error.what()
                << std::endl;
    }
  }
  discarding_backlog = true;
  result.latest_action.reset();
  result.drain_limit_reached = true;
  return result;
}

HighCommandPollResult HighCommandReceiver::ReceiveLatest() noexcept {
  try {
    const bool was_discarding = discarding_backlog_;
    HighCommandPollResult result = DrainHighCommandDatagrams(
        [this]() -> std::optional<std::string> {
          char buffer[kHighCommandDatagramBufferSize]{};
          for (;;) {
            const ssize_t size = ::recv(socket_, buffer, sizeof(buffer), 0);
            if (size >= 0) {
              return std::string(buffer, static_cast<std::size_t>(size));
            }
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) return std::nullopt;
            throw std::runtime_error(
                std::string("high-command UDP receive failed: ") +
                std::strerror(errno));
          }
        },
        max_datagrams_per_poll_, discarding_backlog_);
    if (!was_discarding && result.drain_limit_reached) {
      std::cerr << "high-command UDP backlog exceeded the per-poll budget; "
                   "discarding queued commands until the socket is empty"
                << std::endl;
    }
    return result;
  } catch (const std::exception& error) {
    std::cerr << "high-command UDP drain failed: " << error.what()
              << std::endl;
    return {};
  }
}

}  // namespace a2

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "a2/telemetry.hpp"

namespace {

int failures = 0;

void Check(const bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

int LoopbackSocket(std::uint16_t* port) {
  const int socket_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd < 0) throw std::runtime_error("socket failed");
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(0);
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(socket_fd, reinterpret_cast<const sockaddr*>(&address),
             sizeof(address)) != 0) {
    ::close(socket_fd);
    throw std::runtime_error("bind failed");
  }
  socklen_t length = sizeof(address);
  if (::getsockname(socket_fd, reinterpret_cast<sockaddr*>(&address),
                    &length) != 0) {
    ::close(socket_fd);
    throw std::runtime_error("getsockname failed");
  }
  *port = ntohs(address.sin_port);
  return socket_fd;
}

a2::A2TelemetrySample Sample() {
  a2::A2TelemetrySample sample;
  sample.state_sequence = 12;
  sample.state_capture_time_ns = 1'000'000'000;
  sample.base_velocity_capture_time_ns = 1'001'000'000;
  sample.policy_commit_time_ns = 1'002'000'000;
  sample.filtered_command = {0.4F, -0.2F, 0.5F};
  sample.projected_gravity_b = {0.0F, 0.0F, -1.0F};
  sample.base_linear_velocity_b = {0.3F, -0.1F, 0.0F};
  sample.base_angular_velocity_b = {0.0F, 0.0F, 0.2F};
  for (std::size_t i = 0; i < sample.low_level_policy_action.size(); ++i) {
    sample.low_level_policy_action[i] = static_cast<float>(i) / 10.0F;
  }
  sample.base_velocity_source = "unitree_sport_state_body";
  sample.motion_permitted = true;
  sample.phase = "CTRL";
  return sample;
}

void TestLoopbackAndSequence() {
  std::uint16_t port = 0;
  const int receiver = LoopbackSocket(&port);
  timeval timeout{1, 0};
  ::setsockopt(receiver, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  a2::A2TelemetryProducer producer("127.0.0.1", port);
  Check(producer.healthy(), "producer starts healthy");
  auto sample = Sample();
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const auto now_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
  sample.state_capture_time_ns = now_ns - 3'000'000;
  sample.base_velocity_capture_time_ns = now_ns - 2'000'000;
  sample.policy_commit_time_ns = now_ns - 1'000'000;
  Check(producer.Publish(sample), "valid sample publishes");
  std::array<char, 4096> payload{};
  const ssize_t received = ::recv(receiver, payload.data(), payload.size(), 0);
  Check(received > 0, "loopback receiver gets one packet");
  if (received > 0) {
    const std::string text(payload.data(), static_cast<std::size_t>(received));
    Check(text.find("\"schema\":\"A2TEL1\"") != std::string::npos,
          "packet contains A2TEL1 schema");
    Check(text.find("\"sequence\":0") != std::string::npos,
          "first packet starts at sequence zero");
  }
  ::close(receiver);
}

void TestValidationLatches() {
  std::uint16_t port = 0;
  const int receiver = LoopbackSocket(&port);
  a2::A2TelemetryProducer producer("127.0.0.1", port);
  auto sample = Sample();
  sample.filtered_command[0] = 2.0F;
  Check(!producer.Publish(sample), "command envelope is enforced");
  Check(!producer.healthy(), "validation failure latches producer");
  sample = Sample();
  Check(!producer.Publish(sample), "latched producer rejects later sample");
  ::close(receiver);
}

void TestEndpointValidation() {
  try {
    a2::A2TelemetryProducer invalid("192.0.2.1", 15001);
    Check(false, "non-loopback endpoint is rejected");
  } catch (const std::exception&) {
  }
}

}  // namespace

int main() {
  try {
    TestLoopbackAndSequence();
    TestValidationLatches();
    TestEndpointValidation();
  } catch (const std::exception& error) {
    std::cerr << "FAIL: telemetry test setup: " << error.what() << '\n';
    return 1;
  }
  if (failures != 0) return 1;
  std::cout << "A2 telemetry tests passed\n";
  return 0;
}

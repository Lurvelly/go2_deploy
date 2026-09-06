#include "a2/telemetry.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace a2 {
namespace {

using Clock = std::chrono::steady_clock;

std::uint64_t MonotonicNowNs() noexcept {
  const auto duration = Clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count());
}

bool ValidToken(const std::string& value) noexcept {
  if (value.empty() || value.size() > 64 ||
      (value.front() < 'a' || value.front() > 'z')) {
    return false;
  }
  for (const char character : value) {
    if (!((character >= 'a' && character <= 'z') ||
          (character >= '0' && character <= '9') || character == '_')) {
      return false;
    }
  }
  return true;
}

bool ValidPhase(const std::string& value) noexcept {
  return value == "PREARM" || value == "DAMPING" || value == "STAND" ||
         value == "CTRL" || value == "FAULT" || value == "STOPPING" ||
         value == "STOPPED";
}

bool ValidFault(const std::string& value) noexcept {
  if (value.empty()) return true;
  if (value.size() > 128) return false;
  for (const char character : value) {
    if (character == ',' || character == '\n' || character == '\r' ||
        character == '"' || static_cast<unsigned char>(character) < 0x20) {
      return false;
    }
  }
  return true;
}

void AppendJsonString(std::ostringstream& output, const std::string& value) {
  output << '"';
  for (const char character : value) {
    switch (character) {
      case '"':
        output << "\\\"";
        break;
      case '\\':
        output << "\\\\";
        break;
      case '\b':
        output << "\\b";
        break;
      case '\f':
        output << "\\f";
        break;
      case '\n':
        output << "\\n";
        break;
      case '\r':
        output << "\\r";
        break;
      case '\t':
        output << "\\t";
        break;
      default:
        output << character;
        break;
    }
  }
  output << '"';
}

template <std::size_t N>
void AppendArray(std::ostringstream& output, const std::array<float, N>& value) {
  output << '[' << std::setprecision(9);
  for (std::size_t i = 0; i < N; ++i) {
    if (i != 0) output << ',';
    output << value[i];
  }
  output << ']';
}

std::string NewSessionId() {
  std::random_device random;
  std::array<std::uint32_t, 4> words{};
  for (auto& word : words) word = random();
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (const auto word : words) output << std::setw(8) << word;
  return output.str();
}

}  // namespace

A2TelemetryProducer::A2TelemetryProducer(const std::string& host,
                                         const std::uint16_t port)
    : port_(port), session_id_(NewSessionId()) {
  if (port < 1024) {
    throw std::invalid_argument("A2 telemetry port must be in [1024, 65535]");
  }
  in_addr address{};
  if (inet_pton(AF_INET, host.c_str(), &address) != 1 ||
      (ntohl(address.s_addr) >> 24U) != 127U) {
    throw std::invalid_argument("A2 telemetry destination must be IPv4 loopback");
  }
  destination_address_ = address.s_addr;
  socket_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (socket_ < 0) {
    throw std::runtime_error("cannot create A2 telemetry UDP socket");
  }
}

A2TelemetryProducer::~A2TelemetryProducer() { Close(); }

bool A2TelemetryProducer::Validate(const A2TelemetrySample& sample) const noexcept {
  if (sample.state_sequence == 0 || sample.state_capture_time_ns == 0 ||
      sample.base_velocity_capture_time_ns == 0 ||
      sample.policy_commit_time_ns == 0 ||
      sample.state_capture_time_ns > sample.policy_commit_time_ns ||
      sample.base_velocity_capture_time_ns > sample.policy_commit_time_ns ||
      !AllFinite(sample.filtered_command) ||
      !AllFinite(sample.projected_gravity_b) ||
      !AllFinite(sample.base_linear_velocity_b) ||
      !AllFinite(sample.base_angular_velocity_b) ||
      !AllFinite(sample.low_level_policy_action) ||
      !ValidToken(sample.base_velocity_source) || !ValidPhase(sample.phase) ||
      !ValidFault(sample.fault)) {
    return false;
  }
  const float gravity_norm = std::sqrt(
      sample.projected_gravity_b[0] * sample.projected_gravity_b[0] +
      sample.projected_gravity_b[1] * sample.projected_gravity_b[1] +
      sample.projected_gravity_b[2] * sample.projected_gravity_b[2]);
  if (!std::isfinite(gravity_norm) || gravity_norm < 0.8F ||
      gravity_norm > 1.2F) {
    return false;
  }
  constexpr Vec3 lower{-0.5F, -0.5F, -1.0F};
  constexpr Vec3 upper{1.0F, 0.5F, 1.0F};
  for (std::size_t i = 0; i < lower.size(); ++i) {
    if (sample.filtered_command[i] < lower[i] ||
        sample.filtered_command[i] > upper[i]) {
      return false;
    }
  }
  for (const float value : sample.base_linear_velocity_b) {
    if (std::abs(value) > 10.0F) return false;
  }
  for (const float value : sample.base_angular_velocity_b) {
    if (std::abs(value) > 20.0F) return false;
  }
  for (const float value : sample.low_level_policy_action) {
    if (std::abs(value) > 100.0F) return false;
  }
  static constexpr std::array<std::string_view, 9> forbidden{
      "command", "commanded_velocity", "constant_zero",
      "joint_position_proxy", "target_position_proxy", "unknown", "zeros",
      "zero", "commanded"};
  for (const auto value : forbidden) {
    if (sample.base_velocity_source == value) return false;
  }
  return true;
}

std::string A2TelemetryProducer::Encode(const A2TelemetrySample& sample,
                                         const std::uint64_t sequence,
                                         const std::uint64_t publish_time_ns) const {
  std::ostringstream output;
  output << '{';
  output << "\"schema\":\"A2TEL1\",\"clock_domain\":\"host_monotonic\",\"session_id\":";
  AppendJsonString(output, session_id_);
  output << ",\"sequence\":" << sequence
         << ",\"state_sequence\":" << sample.state_sequence
         << ",\"state_capture_time_ns\":" << sample.state_capture_time_ns
         << ",\"base_velocity_capture_time_ns\":"
         << sample.base_velocity_capture_time_ns
         << ",\"policy_commit_time_ns\":" << sample.policy_commit_time_ns
         << ",\"publish_time_ns\":" << publish_time_ns
         << ",\"filtered_command\":";
  AppendArray(output, sample.filtered_command);
  output << ",\"projected_gravity_b\":";
  AppendArray(output, sample.projected_gravity_b);
  output << ",\"base_linear_velocity_b\":";
  AppendArray(output, sample.base_linear_velocity_b);
  output << ",\"base_angular_velocity_b\":";
  AppendArray(output, sample.base_angular_velocity_b);
  output << ",\"low_level_policy_action\":";
  AppendArray(output, sample.low_level_policy_action);
  output << ",\"base_velocity_source\":";
  AppendJsonString(output, sample.base_velocity_source);
  output << ",\"motion_permitted\":"
         << (sample.motion_permitted ? "true" : "false")
         << ",\"phase\":";
  AppendJsonString(output, sample.phase);
  output << ",\"fault\":";
  AppendJsonString(output, sample.fault);
  output << ",\"low_level_contract_id\":\"a2_45d_project_v0\",\"low_level_policy_sha256\":\"886a653beb628ec09b3287b0e0db79279756e535037679f06d05c29d243b6cc8\"}";
  return output.str();
}

bool A2TelemetryProducer::Publish(const A2TelemetrySample& sample) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (closed_ || failed_ || socket_ < 0 || !Validate(sample)) {
    failed_ = true;
    return false;
  }
  const std::uint64_t sequence = next_sequence_++;
  const std::uint64_t publish_time_ns = MonotonicNowNs();
  if (publish_time_ns < sample.policy_commit_time_ns) {
    failed_ = true;
    return false;
  }
  const std::string payload = Encode(sample, sequence, publish_time_ns);
  if (payload.empty() || payload.size() > kA2TelemetryMaxDatagramBytes) {
    failed_ = true;
    return false;
  }
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(port_);
  destination.sin_addr.s_addr = destination_address_;
  const ssize_t sent = ::sendto(socket_, payload.data(), payload.size(),
                                MSG_NOSIGNAL,
                                reinterpret_cast<const sockaddr*>(&destination),
                                sizeof(destination));
  if (sent != static_cast<ssize_t>(payload.size())) {
    failed_ = true;
    return false;
  }
  return true;
}

void A2TelemetryProducer::Close() noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  if (socket_ >= 0) {
    ::close(socket_);
    socket_ = -1;
  }
  closed_ = true;
}

bool A2TelemetryProducer::healthy() const noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  return !closed_ && !failed_ && socket_ >= 0;
}

}  // namespace a2

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "a2/contract.hpp"

namespace a2 {

inline constexpr const char* kA2TelemetrySchema = "A2TEL1";
inline constexpr const char* kA2TelemetryClockDomain = "host_monotonic";
inline constexpr std::size_t kA2TelemetryMaxDatagramBytes = 4096;

// This is deliberately a value object.  The producer does not know how a
// robot estimator works; callers must provide a measured, timestamped body
// velocity and identify its source explicitly.
struct A2TelemetrySample {
  std::uint64_t state_sequence{0};
  std::uint64_t state_capture_time_ns{0};
  std::uint64_t base_velocity_capture_time_ns{0};
  std::uint64_t policy_commit_time_ns{0};
  Vec3 filtered_command{};
  Vec3 projected_gravity_b{};
  Vec3 base_linear_velocity_b{};
  Vec3 base_angular_velocity_b{};
  JointArray low_level_policy_action{};
  std::string base_velocity_source;
  bool motion_permitted{false};
  std::string phase{"CTRL"};
  std::string fault;
};

class A2TelemetryProducer {
 public:
  // Only IPv4 loopback destinations are accepted.  The producer is optional
  // and is never enabled implicitly by the controller.
  A2TelemetryProducer(const std::string& host, std::uint16_t port);
  ~A2TelemetryProducer();

  A2TelemetryProducer(const A2TelemetryProducer&) = delete;
  A2TelemetryProducer& operator=(const A2TelemetryProducer&) = delete;

  // Returns false after any validation or send failure.  A failure is latched
  // so a later packet cannot silently create a sequence gap or reuse a stream.
  bool Publish(const A2TelemetrySample& sample) noexcept;
  void Close() noexcept;

  bool healthy() const noexcept;
  const std::string& session_id() const noexcept { return session_id_; }

 private:
  bool Validate(const A2TelemetrySample& sample) const noexcept;
  std::string Encode(const A2TelemetrySample& sample,
                     std::uint64_t sequence,
                     std::uint64_t publish_time_ns) const;

  int socket_{-1};
  std::uint32_t destination_address_{0};
  std::uint16_t port_{0};
  std::string session_id_;
  mutable std::mutex mutex_;
  std::uint64_t next_sequence_{0};
  bool failed_{false};
  bool closed_{false};
};

}  // namespace a2

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <type_traits>

namespace a2 {

inline constexpr std::uint32_t kCrc32Polynomial = 0x04C11DB7U;
inline constexpr std::uint32_t kCrc32Initial = 0xFFFFFFFFU;

// Unitree's low-level CRC is deliberately not the reflected/zlib CRC-32. It
// consumes native 32-bit words MSB-first with no final XOR.
inline std::uint32_t Crc32Words(const std::uint32_t* words,
                                const std::size_t word_count) noexcept {
  std::uint32_t crc = kCrc32Initial;
  for (std::size_t i = 0; i < word_count; ++i) {
    std::uint32_t bit = 0x80000000U;
    const std::uint32_t data = words[i];
    for (std::size_t j = 0; j < 32; ++j) {
      if ((crc & 0x80000000U) != 0U) {
        crc = (crc << 1U) ^ kCrc32Polynomial;
      } else {
        crc <<= 1U;
      }
      if ((data & bit) != 0U) {
        crc ^= kCrc32Polynomial;
      }
      bit >>= 1U;
    }
  }
  return crc;
}

inline std::uint32_t Crc32Core(const std::uint32_t* words,
                               const std::size_t word_count) noexcept {
  return Crc32Words(words, word_count);
}

inline std::uint32_t crc32_core(const std::uint32_t* words,
                                const std::size_t word_count) noexcept {
  return Crc32Words(words, word_count);
}

// Safe for potentially unaligned DDS storage. byte_count must cover complete
// native-endian words, exactly as the official uint32_t* implementation does.
inline std::uint32_t Crc32BytesAsWords(const void* bytes,
                                      const std::size_t byte_count) {
  if (byte_count % sizeof(std::uint32_t) != 0U) {
    throw std::invalid_argument("Unitree CRC input is not word-aligned");
  }
  const auto* input = static_cast<const unsigned char*>(bytes);
  std::uint32_t crc = kCrc32Initial;
  for (std::size_t offset = 0; offset < byte_count;
       offset += sizeof(std::uint32_t)) {
    std::uint32_t word = 0U;
    std::memcpy(&word, input + offset, sizeof(word));
    // Keep this loop inline so no unaligned uint32_t pointer is formed.
    std::uint32_t bit = 0x80000000U;
    for (std::size_t j = 0; j < 32; ++j) {
      if ((crc & 0x80000000U) != 0U) {
        crc = (crc << 1U) ^ kCrc32Polynomial;
      } else {
        crc <<= 1U;
      }
      if ((word & bit) != 0U) {
        crc ^= kCrc32Polynomial;
      }
      bit >>= 1U;
    }
  }
  return crc;
}

template <typename Message>
std::uint32_t ComputeMessageCrc(const Message& message) {
  static_assert(std::is_trivially_copyable<Message>::value,
                "DDS CRC input must be trivially copyable");
  static_assert(sizeof(Message) % sizeof(std::uint32_t) == 0U,
                "DDS CRC input size must be a multiple of four bytes");
  static_assert(sizeof(Message) >= sizeof(std::uint32_t),
                "DDS CRC input must include a trailing CRC word");
  return Crc32BytesAsWords(&message, sizeof(Message) - sizeof(std::uint32_t));
}

template <typename Message>
bool HasValidTrailingCrc(const Message& message) {
  static_assert(std::is_trivially_copyable<Message>::value,
                "DDS CRC input must be trivially copyable");
  static_assert(sizeof(Message) % sizeof(std::uint32_t) == 0U,
                "DDS CRC input size must be a multiple of four bytes");
  std::uint32_t expected = 0U;
  const auto* bytes = reinterpret_cast<const unsigned char*>(&message);
  std::memcpy(&expected, bytes + sizeof(Message) - sizeof(expected),
              sizeof(expected));
  return ComputeMessageCrc(message) == expected;
}

}  // namespace a2

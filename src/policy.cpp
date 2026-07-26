#include "a2/policy.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

#include <openssl/evp.h>

namespace a2 {
namespace {

bool IsFinite(const float value) { return std::isfinite(value); }

}  // namespace

A2Policy::A2Policy(const A2Config& config) : config_(config) {
  previous_action_.fill(0.0F);
}

std::string A2Policy::Sha256File(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("Cannot open policy for SHA-256: " + path);
  }

  EVP_MD_CTX* raw_context = EVP_MD_CTX_new();
  if (raw_context == nullptr) {
    throw std::runtime_error("EVP_MD_CTX_new failed");
  }
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(
      raw_context, EVP_MD_CTX_free);
  if (EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("EVP_DigestInit_ex(SHA-256) failed");
  }

  std::array<char, 64 * 1024> buffer{};
  while (stream.good()) {
    stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = stream.gcount();
    if (count > 0 &&
        EVP_DigestUpdate(context.get(), buffer.data(),
                         static_cast<std::size_t>(count)) != 1) {
      throw std::runtime_error("EVP_DigestUpdate(SHA-256) failed");
    }
  }
  if (!stream.eof()) {
    throw std::runtime_error("Failed while reading policy: " + path);
  }

  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1) {
    throw std::runtime_error("EVP_DigestFinal_ex(SHA-256) failed");
  }

  std::ostringstream encoded;
  encoded << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < digest_size; ++i) {
    encoded << std::setw(2) << static_cast<unsigned int>(digest[i]);
  }
  return encoded.str();
}

void A2Policy::Load() {
  const std::string actual_hash = Sha256File(config_.model_path.string());
  if (actual_hash != config_.policy_sha256) {
    throw std::runtime_error("Policy SHA-256 mismatch: expected " +
                             config_.policy_sha256 + ", got " + actual_hash);
  }

  try {
    module_ = torch::jit::load(config_.model_path.string(), torch::kCPU);
    module_.eval();
  } catch (const c10::Error& error) {
    throw std::runtime_error(std::string("Failed to load TorchScript policy: ") +
                             error.what_without_backtrace());
  }

  loaded_ = true;
  Reset();
  VerifyProbeOutput();
}

void A2Policy::Reset() { previous_action_.fill(0.0F); }

std::array<float, 12> A2Policy::Forward(
    const std::array<float, 45>& observation) {
  if (!loaded_) {
    throw std::runtime_error("Policy inference requested before Load()");
  }
  if (!std::all_of(observation.begin(), observation.end(), IsFinite)) {
    throw std::runtime_error("Policy observation contains a non-finite value");
  }

  torch::NoGradGuard no_grad;
  auto input = torch::from_blob(
                   const_cast<float*>(observation.data()), {1, 45},
                   torch::TensorOptions().dtype(torch::kFloat32).device(
                       torch::kCPU))
                   .clone();

  torch::Tensor output;
  try {
    output = module_.forward({input}).toTensor().to(torch::kCPU).contiguous();
  } catch (const c10::Error& error) {
    throw std::runtime_error(std::string("TorchScript inference failed: ") +
                             error.what_without_backtrace());
  }
  if (output.scalar_type() != torch::kFloat32 || output.dim() != 2 ||
      output.size(0) != 1 || output.size(1) != 12) {
    std::ostringstream message;
    message << "Policy output must be float32 [1,12], got " << output.sizes();
    throw std::runtime_error(message.str());
  }

  std::array<float, 12> action{};
  const float* values = output.data_ptr<float>();
  std::copy(values, values + action.size(), action.begin());
  if (!std::all_of(action.begin(), action.end(), IsFinite)) {
    throw std::runtime_error("Policy output contains a non-finite value");
  }
  return action;
}

void A2Policy::VerifyProbeOutput() {
  std::array<float, 45> observation{};
  observation[5] = -1.0F;
  const auto output = Forward(observation);
  if (!config_.policy_golden_output.has_value()) return;

  constexpr float kTolerance = 5.0e-5F;
  for (std::size_t i = 0; i < output.size(); ++i) {
    const float expected = (*config_.policy_golden_output)[i];
    if (std::abs(output[i] - expected) > kTolerance) {
      std::ostringstream message;
      message << "Policy golden-output mismatch at index " << i
              << ": expected " << expected << ", got " << output[i];
      throw std::runtime_error(message.str());
    }
  }
}

PolicyResult A2Policy::Infer(
    const std::array<float, 3>& command,
    const std::array<float, 3>& projected_gravity,
    const std::array<float, 3>& body_angular_velocity,
    const std::array<float, 12>& joint_position,
    const std::array<float, 12>& joint_velocity) {
  PolicyResult result;
  ObservationInput input;
  input.command = command;
  input.projected_gravity = projected_gravity;
  input.body_angular_velocity = body_angular_velocity;
  input.joint_position = joint_position;
  input.joint_velocity = joint_velocity;
  input.previous_action = previous_action_;
  result.observation = BuildObservation(
      input, config_.default_q, config_.command_scale,
      config_.angular_velocity_scale, config_.joint_position_scale,
      config_.joint_velocity_scale);

  result.action = Forward(result.observation);
  result.target_q =
      ActionToTarget(result.action, config_.default_q, config_.action_scale);
  previous_action_ = result.action;
  return result;
}

}  // namespace a2

#include <atomic>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <poll.h>
#include <unistd.h>

#include "a2/config.hpp"
#include "a2/controller.hpp"
#include "a2/high_command_receiver.hpp"
#include "a2/navigation.hpp"
#include "a2/policy.hpp"

namespace fs = std::filesystem;

namespace {

volatile std::sig_atomic_t g_signal_requested = 0;

void HandleSignal(int) { g_signal_requested = 1; }

struct CliOptions {
  bool help{false};
  bool check_contract{false};
  bool simulation{false};
  std::optional<std::string> network_interface;
  std::optional<int> domain_id;
  std::optional<fs::path> config;
  std::optional<fs::path> log_directory;
  std::optional<a2::NavigationSource> navigation_source;
  int high_command_port{15000};
  int telemetry_port{15001};
  bool telemetry_enabled{false};
  bool telemetry_confirmed_body_velocity{false};
};

void PrintUsage(const char* program) {
  std::cout
      << "Usage:\n"
      << "  " << program << " --check-contract [--config PATH]\n"
      << "  " << program
      << " --interface IFACE [--domain ID] [--command-source SOURCE]"
         " [--config PATH] [--log-dir PATH]\n"
      << "  " << program
      << " --sim [--interface IFACE] [--domain ID] [--command-source SOURCE]\n"
      << "      [--high-command-port PORT] [--config PATH] [--log-dir PATH]\n\n"
      << "Options:\n"
      << "  --check-contract  Verify config, policy SHA, 45->12 shape and probe output\n"
      << "  --sim             Use an A2 HG DDS simulator instead of hardware services\n"
      << "  --interface       DDS network interface (simulation default: lo)\n"
      << "  --domain          DDS domain ID (hardware default: 0; simulation: 1)\n"
      << "  --config          Path to params/a2.yaml\n"
      << "  --log-dir         Directory in which a timestamped CSV is created\n"
      << "  --command-source  gamepad, terminal, or high (default: terminal in sim; gamepad on hardware)\n"
      << "  --high-command-port  Loopback UDP port for SOURCE=high (default: 15000)\n"
      << "  --telemetry-enable    Publish A2TEL1 telemetry after committed policy inference\n"
      << "  --telemetry-port      Loopback UDP port for A2TEL1 (default: 15001)\n"
      << "  --telemetry-confirm-sport-state-body\n"
      << "                      Confirm target-A2 SportModeState velocity is measured body-frame data\n"
      << "  -h, --help        Show this help\n";
}

int ParseDomainId(const std::string& value) {
  int domain_id = -1;
  const char* const begin = value.data();
  const char* const end = begin + value.size();
  const auto parsed = std::from_chars(begin, end, domain_id);
  if (parsed.ec != std::errc{} || parsed.ptr != end || domain_id < 0 ||
      domain_id > 232) {
    throw std::invalid_argument(
        "DDS domain ID must be an integer in [0, 232]: " + value);
  }
  return domain_id;
}

int ParsePort(const std::string& value) {
  int port = -1;
  const char* const begin = value.data();
  const char* const end = begin + value.size();
  const auto parsed = std::from_chars(begin, end, port);
  if (parsed.ec != std::errc{} || parsed.ptr != end || port < 1024 ||
      port > 65535) {
    throw std::invalid_argument(
        "high-command port must be an integer in [1024, 65535]: " + value);
  }
  return port;
}

a2::NavigationSource ParseNavigationSource(const std::string& value) {
  if (value == "gamepad") return a2::NavigationSource::kGamepad;
  if (value == "terminal") return a2::NavigationSource::kTerminal;
  if (value == "high") return a2::NavigationSource::kHighPolicy;
  throw std::invalid_argument(
      "command source must be gamepad, terminal, or high: " + value);
}

CliOptions ParseCli(int argc, char** argv) {
  CliOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument(argv[i]);
    if (argument == "-h" || argument == "--help") {
      options.help = true;
    } else if (argument == "--check-contract") {
      options.check_contract = true;
    } else if (argument == "--sim") {
      options.simulation = true;
    } else if (argument == "--telemetry-enable") {
      options.telemetry_enabled = true;
    } else if (argument == "--telemetry-confirm-sport-state-body") {
      options.telemetry_confirmed_body_velocity = true;
    } else if (argument == "--interface" || argument == "--config" ||
               argument == "--log-dir" || argument == "--domain" ||
               argument == "--command-source" ||
               argument == "--high-command-port" ||
               argument == "--telemetry-port") {
      if (i + 1 >= argc) {
        throw std::invalid_argument("Missing value after " + argument);
      }
      const std::string value(argv[++i]);
      if (value.empty()) {
        throw std::invalid_argument("Empty value after " + argument);
      }
      if (argument == "--interface") {
        options.network_interface = value;
      } else if (argument == "--domain") {
        options.domain_id = ParseDomainId(value);
      } else if (argument == "--config") {
        options.config = fs::path(value);
      } else if (argument == "--log-dir") {
        options.log_directory = fs::path(value);
      } else if (argument == "--command-source") {
        options.navigation_source = ParseNavigationSource(value);
      } else if (argument == "--telemetry-port") {
        options.telemetry_port = ParsePort(value);
      } else {
        options.high_command_port = ParsePort(value);
      }
    } else {
      throw std::invalid_argument("Unknown argument: " + argument);
    }
  }
  if (!options.help && !options.check_contract && !options.simulation &&
      !options.network_interface.has_value()) {
    throw std::invalid_argument(
        "--interface is required unless --check-contract is used");
  }
  if (options.check_contract &&
      (options.network_interface.has_value() || options.domain_id.has_value() ||
       options.simulation || options.navigation_source.has_value() ||
       options.telemetry_enabled ||
       options.telemetry_confirmed_body_velocity)) {
    throw std::invalid_argument(
        "--check-contract cannot be combined with DDS runtime options");
  }
  if (options.check_contract && options.log_directory.has_value()) {
    throw std::invalid_argument(
        "--log-dir is only valid with --interface");
  }
  if (options.telemetry_confirmed_body_velocity &&
      !options.telemetry_enabled) {
    throw std::invalid_argument(
        "--telemetry-confirm-sport-state-body requires --telemetry-enable");
  }
  if (options.telemetry_enabled &&
      !options.telemetry_confirmed_body_velocity) {
    throw std::invalid_argument(
        "--telemetry-enable requires --telemetry-confirm-sport-state-body; "
        "verify target-A2 body-frame estimator first");
  }
  return options;
}

void PrintTerminalHelp(const a2::NavigationSource source) {
  std::cout
      << "Terminal commands (press Enter):\n"
      << "  arm | stand | ctrl | damp | rearm | zero | quit\n";
  if (source == a2::NavigationSource::kTerminal) {
    std::cout << "  cmd <vx> <vy> <yaw_rate>\n";
  } else if (source == a2::NavigationSource::kHighPolicy) {
    std::cout << "  velocity comes from the A2NAV1 high-policy UDP stream\n";
  } else {
    std::cout << "  velocity comes from the gamepad\n";
  }
}

bool ProcessTerminalCommand(const std::string& line,
                            const a2::NavigationSource source,
                            a2::A2Controller& controller) {
  std::istringstream input(line);
  std::string command;
  input >> command;
  if (command.empty()) return false;

  auto require_end = [&]() {
    std::string trailing;
    if (input >> trailing) {
      throw std::invalid_argument("unexpected argument: " + trailing);
    }
  };

  try {
    if (command == "arm") {
      require_end();
      controller.SubmitOperatorRequest(a2::OperatorRequest::kArmDamping);
    } else if (command == "stand") {
      require_end();
      controller.SubmitOperatorRequest(a2::OperatorRequest::kStand);
    } else if (command == "ctrl") {
      require_end();
      controller.SubmitOperatorRequest(a2::OperatorRequest::kControl);
    } else if (command == "damp") {
      require_end();
      controller.SubmitOperatorRequest(a2::OperatorRequest::kDamping);
    } else if (command == "rearm") {
      require_end();
      controller.SubmitOperatorRequest(a2::OperatorRequest::kRearm);
    } else if (command == "zero") {
      require_end();
      if (source != a2::NavigationSource::kTerminal) {
        throw std::invalid_argument(
            "zero is only available with --command-source terminal");
      }
      controller.SubmitNavigationAction({});
      std::cout << "Terminal navigation action -> [0, 0, 0]" << std::endl;
    } else if (command == "cmd") {
      if (source != a2::NavigationSource::kTerminal) {
        throw std::invalid_argument(
            "cmd is only available with --command-source terminal");
      }
      a2::Vec3 action{};
      if (!(input >> action[0] >> action[1] >> action[2])) {
        throw std::invalid_argument("usage: cmd <vx> <vy> <yaw_rate>");
      }
      require_end();
      controller.SubmitNavigationAction(action);
      std::cout << "Terminal navigation action -> [" << action[0] << ", "
                << action[1] << ", " << action[2] << "]" << std::endl;
    } else if (command == "help") {
      require_end();
      PrintTerminalHelp(source);
    } else if (command == "quit") {
      require_end();
      return true;
    } else {
      throw std::invalid_argument("unknown terminal command: " + command);
    }
  } catch (const std::exception& error) {
    std::cerr << "terminal command rejected: " << error.what() << std::endl;
  }
  return false;
}

fs::path ExecutableDirectory(const char* argv0) {
  std::error_code error;
  fs::path executable = fs::canonical("/proc/self/exe", error);
  if (!error) {
    return executable.parent_path();
  }
  executable = fs::absolute(argv0, error);
  if (!error) {
    return executable.parent_path();
  }
  return fs::current_path();
}

fs::path ResolveDefaultConfig(const char* argv0) {
  const fs::path executable_directory = ExecutableDirectory(argv0);
  const std::vector<fs::path> candidates{
      fs::current_path() / "params" / "a2.yaml",
      executable_directory / "share" / "a2_deploy" / "params" / "a2.yaml",
      executable_directory / ".." / "share" / "a2_deploy" / "params" /
          "a2.yaml",
  };
  for (const auto& candidate : candidates) {
    std::error_code error;
    if (fs::is_regular_file(candidate, error)) {
      return fs::weakly_canonical(candidate);
    }
  }
  std::ostringstream message;
  message << "Cannot locate params/a2.yaml. Tried:";
  for (const auto& candidate : candidates) {
    message << "\n  " << candidate;
  }
  throw std::runtime_error(message.str());
}

std::string Timestamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local{};
#if defined(_WIN32)
  localtime_s(&local, &time);
#else
  localtime_r(&time, &local);
#endif
  std::ostringstream value;
  value << std::put_time(&local, "%Y%m%d-%H%M%S");
  return value.str();
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const CliOptions cli = ParseCli(argc, argv);
    if (cli.help) {
      PrintUsage(argv[0]);
      return 0;
    }

    const fs::path config_path =
        cli.config.has_value() ? fs::absolute(*cli.config)
                               : ResolveDefaultConfig(argv[0]);
    const a2::A2Config config = a2::A2Config::Load(config_path);

    std::cout << "A2 contract: " << config.contract_id << '\n'
              << "Config: " << config.source_path << '\n'
              << "Policy: " << config.model_path << '\n'
              << "Policy SHA-256: " << config.policy_sha256 << std::endl;

    if (cli.check_contract) {
      a2::A2Policy policy(config);
      policy.Load();
      std::cout << "Contract check passed: policy SHA, 45D -> 12D and "
                << (config.policy_golden_output.has_value()
                        ? "configured reference output."
                        : "finite probe output.")
                << std::endl;
      return 0;
    }

    fs::path log_directory =
        cli.log_directory.value_or(fs::current_path() / "logs");
    log_directory = fs::absolute(log_directory) / Timestamp();
    fs::create_directories(log_directory);
    const fs::path log_file = log_directory / "a2_runtime.csv";

    const std::string network_interface =
        cli.network_interface.value_or("lo");
    const int domain_id = cli.domain_id.value_or(cli.simulation ? 1 : 0);
    const a2::NavigationSource navigation_source =
        cli.navigation_source.value_or(
            cli.simulation ? a2::NavigationSource::kTerminal
                           : a2::NavigationSource::kGamepad);
    a2::RuntimeOptions runtime_options;
    runtime_options.network_interface = network_interface;
    runtime_options.log_file = log_file;
    runtime_options.domain_id = domain_id;
    runtime_options.simulation = cli.simulation;
    runtime_options.navigation_source = navigation_source;
    runtime_options.telemetry_enabled = cli.telemetry_enabled;
    runtime_options.telemetry_confirmed_body_velocity =
        cli.telemetry_confirmed_body_velocity;
    runtime_options.telemetry_port =
        static_cast<std::uint16_t>(cli.telemetry_port);
    a2::A2Controller controller(config, runtime_options);

    std::unique_ptr<a2::HighCommandReceiver> high_command_receiver;
    if (navigation_source == a2::NavigationSource::kHighPolicy) {
      high_command_receiver =
          std::make_unique<a2::HighCommandReceiver>(cli.high_command_port);
      std::cout << "High-policy command input: UDP 127.0.0.1:"
                << high_command_receiver->port()
                << " (A2NAV1 vx vy yaw_rate)" << std::endl;
    }
    if (cli.simulation ||
        navigation_source == a2::NavigationSource::kTerminal) {
      PrintTerminalHelp(navigation_source);
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    std::atomic<bool> completed{false};
    int runtime_result = 1;
    std::exception_ptr runtime_error;
    std::thread runtime([&]() {
      try {
        runtime_result = controller.Run();
      } catch (...) {
        runtime_error = std::current_exception();
      }
      completed.store(true, std::memory_order_release);
    });

    // Hardware terminal control uses the same narrow navigation and operator
    // request APIs as simulation. It never gives the terminal direct LowCmd
    // or SafetySupervisor access.
    bool terminal_input_open =
        cli.simulation || navigation_source == a2::NavigationSource::kTerminal;
    bool terminal_requested_shutdown = false;
    while (!completed.load(std::memory_order_acquire) &&
           g_signal_requested == 0 && !terminal_requested_shutdown) {
      if (high_command_receiver) {
        const auto received = high_command_receiver->ReceiveLatest();
        if (received.latest_action) {
          controller.SubmitNavigationAction(*received.latest_action);
        }
      }

      if (terminal_input_open) {
        pollfd input_poll{STDIN_FILENO, POLLIN | POLLHUP, 0};
        const int poll_result = ::poll(&input_poll, 1, 0);
        if (poll_result > 0 && (input_poll.revents & POLLIN) != 0) {
          std::string line;
          if (std::getline(std::cin, line)) {
            terminal_requested_shutdown =
                ProcessTerminalCommand(line, navigation_source, controller);
          } else {
            terminal_input_open = false;
          }
        } else if (poll_result > 0 && (input_poll.revents & POLLHUP) != 0) {
          terminal_input_open = false;
        } else if (poll_result < 0 && errno != EINTR) {
          std::cerr << "terminal input poll failed: " << std::strerror(errno)
                    << std::endl;
          terminal_input_open = false;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (g_signal_requested != 0 || terminal_requested_shutdown) {
      std::cout << (g_signal_requested != 0 ? "Shutdown signal received"
                                            : "Terminal quit requested")
                << "; entering controlled stop." << std::endl;
      controller.RequestShutdown();
    }
    runtime.join();
    if (runtime_error) {
      std::rethrow_exception(runtime_error);
    }
    std::cout << "Log: " << log_file << std::endl;
    return runtime_result;
  } catch (const std::exception& error) {
    std::cerr << "a2_deploy: " << error.what() << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }
}

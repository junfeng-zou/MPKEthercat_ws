#include "mpk_soem/four_motor_master.hpp"

#include <array>
#include <atomic>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

std::atomic_bool g_stop_requested{false};

void signal_handler(int)
{
  g_stop_requested.store(true);
}

[[noreturn]] void usage(const char * program, int exit_code)
{
  std::cerr
    << "Usage:\n"
    << "  " << program << " --ifname <ethX> [options]\n\n"
    << "Options:\n"
    << "  --mode <csp|csv|pp|pv|idle>       CiA402 mode, default csp\n"
    << "  --target <v1,v2,v3,v4>            Target counts or velocity units\n"
    << "  --enable                          Walk all drives to Operation Enabled\n"
    << "  --enable-mask <mask>               Enabled motor bitmask, default 0x0f\n"
    << "  --duration <seconds>              Stop after N seconds, default forever\n"
    << "  --rate-hz <hz>                    Cycle rate, default 1000\n"
    << "  --csp-speed <counts/s>            CSP ramp limit, default immediate\n"
    << "  --hold-current                    Start position modes at actual positions\n"
    << "  --print-every <cycles>            Status print period, default 1000\n"
    << "  --no-dc                           Do not enable DC SYNC0\n"
    << "  --no-id-check                     Skip vendor/product identity check\n"
    << "  --help                            Show this help\n\n"
    << "Examples:\n"
    << "  sudo " << program << " --ifname enp3s0 --mode csp --target 0,0,0,0 --enable\n"
    << "  sudo " << program << " --ifname enp3s0 --mode csv --target 100,0,0,0 --enable --duration 5\n";
  std::exit(exit_code);
}

std::vector<std::string> split(const std::string & text, char sep)
{
  std::vector<std::string> parts;
  std::stringstream ss(text);
  std::string item;
  while (std::getline(ss, item, sep)) {
    parts.push_back(item);
  }
  return parts;
}

std::array<int32_t, mpk_soem::kMaxMotors> parse_targets(const std::string & text)
{
  std::array<int32_t, mpk_soem::kMaxMotors> values{0, 0, 0, 0};
  const auto parts = split(text, ',');
  if (parts.empty() || parts.size() > mpk_soem::kMaxMotors) {
    throw std::invalid_argument("--target expects 1 to 4 comma-separated values");
  }
  for (size_t i = 0; i < parts.size(); ++i) {
    values[i] = static_cast<int32_t>(std::stol(parts[i]));
  }
  if (parts.size() == 1) {
    values.fill(values[0]);
  }
  return values;
}

mpk_soem::MasterOptions parse_args(int argc, char ** argv)
{
  mpk_soem::MasterOptions options;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto require_value = [&](const std::string & key) -> std::string {
      if (i + 1 >= argc) {
        throw std::invalid_argument("missing value for " + key);
      }
      return argv[++i];
    };

    if (arg == "--ifname") {
      options.ifname = require_value(arg);
    } else if (arg == "--mode") {
      options.mode = mpk_soem::cia402::parse_mode(require_value(arg));
    } else if (arg == "--target") {
      options.targets = parse_targets(require_value(arg));
    } else if (arg == "--enable") {
      options.enable_drives = true;
    } else if (arg == "--enable-mask") {
      const auto mask = std::stoul(require_value(arg), nullptr, 0);
      if (mask == 0 || mask > 0x0f) {
        throw std::invalid_argument("--enable-mask expects a value from 0x1 to 0x0f");
      }
      options.enable_motor_mask = static_cast<uint8_t>(mask);
    } else if (arg == "--duration") {
      options.duration_s = std::stod(require_value(arg));
    } else if (arg == "--rate-hz") {
      const double hz = std::stod(require_value(arg));
      if (hz <= 0.0) {
        throw std::invalid_argument("--rate-hz must be positive");
      }
      options.cycle_time_ns = static_cast<int64_t>(1000000000.0 / hz);
      options.print_every_cycles = static_cast<int>(hz);
    } else if (arg == "--csp-speed") {
      options.csp_speed_counts_per_s = std::stod(require_value(arg));
    } else if (arg == "--hold-current") {
      options.hold_current_on_start = true;
    } else if (arg == "--print-every") {
      options.print_every_cycles = std::stoi(require_value(arg));
    } else if (arg == "--no-dc") {
      options.use_distributed_clocks = false;
    } else if (arg == "--no-id-check") {
      options.require_expected_identity = false;
    } else if (arg == "--help" || arg == "-h") {
      usage(argv[0], 0);
    } else {
      throw std::invalid_argument("unknown argument: " + arg);
    }
  }

  if (options.ifname.empty()) {
    throw std::invalid_argument("--ifname is required");
  }
  return options;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  try {
    auto options = parse_args(argc, argv);
    std::cout << "MPK SOEM four-motor controller: mode="
              << mpk_soem::cia402::to_string(options.mode)
              << ", cycle=" << options.cycle_time_ns << " ns"
              << ", enable=" << (options.enable_drives ? "true" : "false")
              << std::endl;

    mpk_soem::FourMotorMaster master(options);
    if (!master.initialize()) {
      return 1;
    }
    return master.run(g_stop_requested);
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << "\n\n";
    usage(argv[0], 2);
  }
}

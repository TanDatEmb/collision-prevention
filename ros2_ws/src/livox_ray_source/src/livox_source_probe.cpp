#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "livox_ray_source/livox_sdk_sample_source.hpp"

namespace
{

struct Options
{
  std::string config;
  std::string primary_serial;
  double duration_s{10.0};
  bool require_ptp{true};
  bool configure_point_type{true};
};

void usage(const char * program)
{
  std::cout << "Usage: " << program
            << " --config PATH [--duration-s SECONDS] [--primary-sn SERIAL]"
               " [--allow-unsynchronized] [--no-configure-point-type]\n";
}

Options parse(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    const auto next = [&]() -> std::string {
      if (index + 1 >= argc) {
        throw std::invalid_argument("missing value for " + argument);
      }
      return argv[++index];
    };
    if (argument == "--help") {
      usage(argv[0]);
      std::exit(0);
    }
    if (argument == "--config") {
      options.config = next();
    }
    else if (argument == "--duration-s") {
      options.duration_s = std::stod(next());
    }
    else if (argument == "--primary-sn") {
      options.primary_serial = next();
    }
    else if (argument == "--allow-unsynchronized") {
      options.require_ptp = false;
    }
    else if (argument == "--no-configure-point-type") {
      options.configure_point_type = false;
    }
    else {
      throw std::invalid_argument("unknown argument: " + argument);
    }
  }
  if (options.config.empty()) {
    throw std::invalid_argument("--config is required");
  }
  if (!std::isfinite(options.duration_s) || options.duration_s <= 0.0) {
    throw std::invalid_argument("--duration-s must be positive and finite");
  }
  return options;
}

std::string jsonString(const std::string & value)
{
  std::string result;
  result.reserve(value.size() + 2U);
  result.push_back('"');
  for (const char character : value) {
    if (character == '\\' || character == '"') {
      result.push_back('\\');
    }
    result.push_back(character);
  }
  result.push_back('"');
  return result;
}

void add(std::ostringstream & output, const char * key, uint64_t value, bool & first)
{
  if (!first) {
    output << ',';
  }
  first = false;
  output << jsonString(key) << ':' << value;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parse(argc, argv);
    std::atomic<uint64_t> stream_samples{0U};
    std::atomic<uint64_t> point_samples{0U};
    std::atomic<uint64_t> imu_samples{0U};

    livox_ray_source::LivoxSdkSampleSource source(
      options.config,
      [&stream_samples](const livox_ray_source::StreamSample &) { ++stream_samples; },
      options.configure_point_type,
      [](const std::string & message) { std::cerr << message << '\n'; }, options.primary_serial);
    source.setPointSink(
      [&point_samples](const livox_ray_source::LivoxPointSample &) { ++point_samples; });
    source.setImuSink([&imu_samples](const livox_ray_source::LivoxImuSample &) { ++imu_samples; });
    source.setGyroSink([](const livox_ray_source::GyroSample &) {});

    if (!source.start()) {
      throw std::runtime_error("Livox SDK did not start");
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(options.duration_s));
    source.stop();

    const auto stats = source.stats();
    const auto health = source.health();
    const bool qualified =
      stats.packets > 0U && stats.decoded_packets > 0U && stream_samples > 0U &&
      point_samples > 0U && imu_samples > 0U && stats.rejected_packets == 0U &&
      stats.not_spherical_packets == 0U && stats.continuity_breaks == 0U &&
      stats.dropped_rays == 0U && stats.imu_unusable == 0U && stats.imu_accel_unusable == 0U &&
      (!options.require_ptp || stats.unsynchronized_packets == 0U) && !source.multipleLidarFault();

    std::ostringstream output;
    output << '{';
    bool first = true;
    output << jsonString("config") << ':' << jsonString(options.config);
    first = false;
    add(output, "duration_ms", static_cast<uint64_t>(options.duration_s * 1000.0), first);
    add(output, "stream_samples", stream_samples.load(), first);
    add(output, "point_samples", point_samples.load(), first);
    add(output, "imu_samples", imu_samples.load(), first);
    add(output, "packets", stats.packets, first);
    add(output, "decoded_packets", stats.decoded_packets, first);
    add(output, "rejected_packets", stats.rejected_packets, first);
    add(output, "not_spherical_packets", stats.not_spherical_packets, first);
    add(output, "unsynchronized_packets", stats.unsynchronized_packets, first);
    add(output, "continuity_breaks", stats.continuity_breaks, first);
    add(output, "dropped_rays", stats.dropped_rays, first);
    add(output, "point_metadata_samples", stats.point_metadata_samples, first);
    add(output, "imu_packets", stats.imu_packets, first);
    add(output, "imu_unusable", stats.imu_unusable, first);
    add(output, "imu_accel_unusable", stats.imu_accel_unusable, first);
    add(output, "device_events", stats.device_events, first);
    add(output, "non_primary_samples", stats.non_primary_samples, first);
    add(output, "device_count", health.size(), first);
    output << ',' << jsonString("multiple_lidar_fault") << ':'
           << (source.multipleLidarFault() ? "true" : "false");
    output << ',' << jsonString("qualified") << ':' << (qualified ? "true" : "false");
    output << "}\n";
    std::cout << output.str();
    return qualified ? 0 : 3;
  }
  catch (const std::exception & error) {
    std::cerr << "livox_source_probe: " << error.what() << '\n';
    return 2;
  }
}

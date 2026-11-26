#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace livox_ray_source
{

/// Point layouts accepted by the Mid-360S SDK2 stream.
enum class Mid360SPointDataType : uint8_t
{
  kSpherical,
  kCartesianHigh,
  kCartesianLow,
};

enum class Mid360SScanPattern : uint8_t
{
  kNonRepetitive,
  kRepetitive,
  kRepetitiveLowFrameRate,
};

enum class Mid360SPpsSyncMode : uint8_t
{
  kNormal,
  kSpecial,
};

enum class Mid360SImuRate : uint8_t
{
  k200Hz,
  k500Hz,
  k100Hz,
  k50Hz,
};

enum class Mid360SAccelRange : uint8_t
{
  k4G,
  k8G,
  k16G,
  k32G,
};

enum class Mid360SGyroRange : uint8_t
{
  k2000Dps,
  k1000Dps,
  k500Dps,
  k250Dps,
  k125Dps,
  k62_5Dps,
  k31_25Dps,
  k15_625Dps,
};

struct Mid360SImuRange
{
  Mid360SImuRate output_rate{Mid360SImuRate::k200Hz};
  Mid360SAccelRange accel_range{Mid360SAccelRange::k4G};
  Mid360SGyroRange gyro_range{Mid360SGyroRange::k2000Dps};
};

struct Mid360SInstallAttitude
{
  float roll_deg{0.0F};
  float pitch_deg{0.0F};
  float yaw_deg{0.0F};
  int32_t x_mm{0};
  int32_t y_mm{0};
  int32_t z_mm{0};
};

/// Mid-360S command settings used by the adapter.
struct Mid360SConfig
{
  Mid360SPointDataType point_data_type{Mid360SPointDataType::kSpherical};
  std::optional<Mid360SScanPattern> scan_pattern;
  std::optional<uint32_t> blind_spot_mm;
  std::optional<bool> dual_emit;
  std::optional<Mid360SPpsSyncMode> pps_sync_mode;
  std::optional<Mid360SImuRange> imu_range;
  std::optional<Mid360SInstallAttitude> install_attitude;
};

/// Parses the adapter settings from the Livox SDK2 JSON configuration.
/// Throws `std::invalid_argument` for malformed values or unsupported enum strings.
[[nodiscard]] Mid360SConfig parseMid360SConfig(const std::string & json_text);

/// Loads and parses adapter settings from the SDK configuration file.
[[nodiscard]] Mid360SConfig loadMid360SConfig(const std::string & path);

}  // namespace livox_ray_source

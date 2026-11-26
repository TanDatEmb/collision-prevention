#include "livox_ray_source/mid360s_config.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

namespace livox_ray_source
{
namespace
{

std::invalid_argument invalid(const std::string & key, const std::string & value)
{
  return std::invalid_argument(
    "collision_prevention." + key + " has unsupported value '" + value + "'");
}

std::string scalarString(const YAML::Node & node, const std::string & key)
{
  if (!node.IsScalar()) {
    throw std::invalid_argument("collision_prevention." + key + " must be a scalar");
  }
  try {
    return node.as<std::string>();
  }
  catch (const YAML::Exception &) {
    throw std::invalid_argument("collision_prevention." + key + " must be a string");
  }
}

template <typename T>
T scalar(const YAML::Node & node, const std::string & key)
{
  try {
    return node.as<T>();
  }
  catch (const YAML::Exception &) {
    throw std::invalid_argument("collision_prevention." + key + " has an invalid value");
  }
}

float finiteFloat(const YAML::Node & node, const std::string & key)
{
  const double value = scalar<double>(node, key);
  if (!std::isfinite(value) || std::abs(value) > std::numeric_limits<float>::max()) {
    throw std::invalid_argument("collision_prevention." + key + " must be finite");
  }
  return static_cast<float>(value);
}

int32_t millimetres(const YAML::Node & node, const std::string & key)
{
  const int64_t value = scalar<int64_t>(node, key);
  if (value < std::numeric_limits<int32_t>::min() || value > std::numeric_limits<int32_t>::max()) {
    throw std::invalid_argument("collision_prevention." + key + " is out of range");
  }
  return static_cast<int32_t>(value);
}

Mid360SPointDataType pointType(const YAML::Node & node)
{
  const std::string value = scalarString(node, "point_data_type");
  if (value == "spherical") {
    return Mid360SPointDataType::kSpherical;
  }
  if (value == "cartesian_high") {
    return Mid360SPointDataType::kCartesianHigh;
  }
  if (value == "cartesian_low") {
    return Mid360SPointDataType::kCartesianLow;
  }
  throw invalid("point_data_type", value);
}

Mid360SScanPattern scanPattern(const YAML::Node & node)
{
  const std::string value = scalarString(node, "scan_pattern");
  if (value == "non_repetitive") {
    return Mid360SScanPattern::kNonRepetitive;
  }
  if (value == "repetitive") {
    return Mid360SScanPattern::kRepetitive;
  }
  if (value == "repetitive_low_frame_rate") {
    return Mid360SScanPattern::kRepetitiveLowFrameRate;
  }
  throw invalid("scan_pattern", value);
}

Mid360SPpsSyncMode ppsSync(const YAML::Node & node)
{
  const std::string value = scalarString(node, "pps_sync");
  if (value == "normal") {
    return Mid360SPpsSyncMode::kNormal;
  }
  if (value == "special") {
    return Mid360SPpsSyncMode::kSpecial;
  }
  throw invalid("pps_sync", value);
}

Mid360SImuRate imuRate(const YAML::Node & node)
{
  const uint32_t value = scalar<uint32_t>(node, "imu.rate_hz");
  switch (value) {
    case 200U:
      return Mid360SImuRate::k200Hz;
    case 500U:
      return Mid360SImuRate::k500Hz;
    case 100U:
      return Mid360SImuRate::k100Hz;
    case 50U:
      return Mid360SImuRate::k50Hz;
    default:
      throw invalid("imu.rate_hz", std::to_string(value));
  }
}

Mid360SAccelRange accelRange(const YAML::Node & node)
{
  const uint32_t value = scalar<uint32_t>(node, "imu.accel_range_g");
  switch (value) {
    case 4U:
      return Mid360SAccelRange::k4G;
    case 8U:
      return Mid360SAccelRange::k8G;
    case 16U:
      return Mid360SAccelRange::k16G;
    case 32U:
      return Mid360SAccelRange::k32G;
    default:
      throw invalid("imu.accel_range_g", std::to_string(value));
  }
}

Mid360SGyroRange gyroRange(const YAML::Node & node)
{
  const double value = scalar<double>(node, "imu.gyro_range_dps");
  if (value == 2000.0) return Mid360SGyroRange::k2000Dps;
  if (value == 1000.0) return Mid360SGyroRange::k1000Dps;
  if (value == 500.0) return Mid360SGyroRange::k500Dps;
  if (value == 250.0) return Mid360SGyroRange::k250Dps;
  if (value == 125.0) return Mid360SGyroRange::k125Dps;
  if (value == 62.5) return Mid360SGyroRange::k62_5Dps;
  if (value == 31.25) return Mid360SGyroRange::k31_25Dps;
  if (value == 15.625) return Mid360SGyroRange::k15_625Dps;
  throw invalid("imu.gyro_range_dps", std::to_string(value));
}

Mid360SInstallAttitude installAttitude(const YAML::Node & node)
{
  if (!node.IsMap()) {
    throw std::invalid_argument("collision_prevention.install_attitude must be an object");
  }
  const char * const required[] = {"roll_deg", "pitch_deg", "yaw_deg", "x_mm", "y_mm", "z_mm"};
  for (const char * key : required) {
    if (!node[key]) {
      throw std::invalid_argument(
        "collision_prevention.install_attitude needs roll_deg, pitch_deg, yaw_deg, x_mm, y_mm and "
        "z_mm");
    }
  }
  return Mid360SInstallAttitude{
    finiteFloat(node["roll_deg"], "install_attitude.roll_deg"),
    finiteFloat(node["pitch_deg"], "install_attitude.pitch_deg"),
    finiteFloat(node["yaw_deg"], "install_attitude.yaw_deg"),
    millimetres(node["x_mm"], "install_attitude.x_mm"),
    millimetres(node["y_mm"], "install_attitude.y_mm"),
    millimetres(node["z_mm"], "install_attitude.z_mm")};
}

}  // namespace

Mid360SConfig parseMid360SConfig(const std::string & json_text)
{
  if (json_text.empty()) {
    throw std::invalid_argument("Mid-360S configuration is empty");
  }

  YAML::Node root;
  try {
    root = YAML::Load(json_text);
  }
  catch (const YAML::Exception & error) {
    throw std::invalid_argument(std::string("Mid-360S configuration is invalid: ") + error.what());
  }
  if (!root.IsMap()) {
    throw std::invalid_argument("Mid-360S configuration root must be an object");
  }

  Mid360SConfig result;
  const YAML::Node project = root["collision_prevention"];
  if (!project) {
    return result;
  }
  if (!project.IsMap()) {
    throw std::invalid_argument("collision_prevention must be an object");
  }

  if (project["point_data_type"]) {
    result.point_data_type = pointType(project["point_data_type"]);
  }
  if (project["scan_pattern"]) {
    result.scan_pattern = scanPattern(project["scan_pattern"]);
  }
  if (project["blind_spot_mm"]) {
    const uint64_t value = scalar<uint64_t>(project["blind_spot_mm"], "blind_spot_mm");
    if (value > std::numeric_limits<uint32_t>::max()) {
      throw std::invalid_argument("collision_prevention.blind_spot_mm is too large");
    }
    result.blind_spot_mm = static_cast<uint32_t>(value);
  }
  if (project["dual_emit"]) {
    result.dual_emit = scalar<bool>(project["dual_emit"], "dual_emit");
  }
  if (project["pps_sync"]) {
    result.pps_sync_mode = ppsSync(project["pps_sync"]);
  }
  if (project["imu"]) {
    const YAML::Node imu = project["imu"];
    if (!imu.IsMap()) {
      throw std::invalid_argument("collision_prevention.imu must be an object");
    }
    if (!imu["rate_hz"] || !imu["accel_range_g"] || !imu["gyro_range_dps"]) {
      throw std::invalid_argument(
        "collision_prevention.imu needs rate_hz, accel_range_g and gyro_range_dps");
    }
    result.imu_range = Mid360SImuRange{
      imuRate(imu["rate_hz"]), accelRange(imu["accel_range_g"]), gyroRange(imu["gyro_range_dps"])};
  }
  if (project["install_attitude"]) {
    result.install_attitude = installAttitude(project["install_attitude"]);
  }
  return result;
}

Mid360SConfig loadMid360SConfig(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::invalid_argument("cannot open Mid-360S configuration: " + path);
  }
  std::ostringstream contents;
  contents << file.rdbuf();
  return parseMid360SConfig(contents.str());
}

}  // namespace livox_ray_source

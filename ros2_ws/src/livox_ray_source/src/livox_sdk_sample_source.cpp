#include "livox_ray_source/livox_sdk_sample_source.hpp"

#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

namespace livox_ray_source
{

namespace
{

// The SDK keeps global state: one running instance.
std::atomic<bool> g_instance_running{false};

}  // namespace

namespace detail
{

struct SdkSourceState
{
  std::string config_path;
  LivoxSdkSampleSource::Sink sink;
  LivoxSdkSampleSource::Log log;
  LivoxSdkSampleSource::GyroSink gyro_sink;
  bool configure_point_type{true};
  bool started{false};
  std::atomic<bool> detection_handled{false};
  UdpContinuityTracker tracker;
  PacketSampler sampler;

  std::atomic<uint64_t> packets{0U};
  std::atomic<uint64_t> decoded_packets{0U};
  std::atomic<uint64_t> rejected_packets{0U};
  std::atomic<uint64_t> not_spherical_packets{0U};
  std::atomic<uint64_t> unsynchronized_packets{0U};
  std::atomic<uint64_t> continuity_breaks{0U};
  std::atomic<uint64_t> dropped_rays{0U};
  std::atomic<uint64_t> imu_packets{0U};
  std::atomic<uint64_t> imu_unusable{0U};

  void say(const std::string & text) const
  {
    if (log) {
      log(text);
    }
  }
};

}  // namespace detail

namespace
{

using Impl = detail::SdkSourceState;

void onPointCloud(uint32_t, const uint8_t, LivoxLidarEthernetPacket * packet, void * client_data)
{
  if (packet == nullptr || client_data == nullptr) {
    return;
  }
  auto * impl = static_cast<Impl *>(client_data);
  ++impl->packets;
  // `length` is the size of the whole UDP payload (checked on a real Mid-360S: it equals the
  // datagram size for every packet).
  const DecodedPacket decoded =
    decodeSphericalPacket(reinterpret_cast<const uint8_t *>(packet), packet->length);
  if (decoded.status == DecodeStatus::kNotSpherical) {
    ++impl->not_spherical_packets;
    return;
  }
  if (decoded.status != DecodeStatus::kOk) {
    ++impl->rejected_packets;
    return;
  }
  ++impl->decoded_packets;
  if (decoded.time_type != 1U) {
    ++impl->unsynchronized_packets;
  }
  const bool is_break = impl->tracker.update(decoded.udp_cnt);
  if (is_break) {
    ++impl->continuity_breaks;
  }
  std::vector<StreamSample> samples;
  samples.reserve(decoded.rays.size());
  impl->dropped_rays += impl->sampler.append(decoded, is_break, samples);
  for (const auto & sample : samples) {
    impl->sink(sample);
  }
}

void onImu(const uint32_t, const uint8_t, LivoxLidarEthernetPacket * packet, void * client_data)
{
  if (packet == nullptr || client_data == nullptr) {
    return;
  }
  auto * impl = static_cast<Impl *>(client_data);
  ++impl->imu_packets;
  const DecodedImuPacket decoded =
    decodeImuPacket(reinterpret_cast<const uint8_t *>(packet), packet->length);
  if (!decoded.gyro.isValid()) {
    ++impl->imu_unusable;
    return;
  }
  impl->gyro_sink(decoded.gyro);
}

int pointTypeOf(const LivoxLidarDiagInternalInfoResponse & response)
{
  uint16_t offset = 0U;
  for (uint16_t index = 0U; index < response.param_num; ++index) {
    const auto * pair = reinterpret_cast<const LivoxLidarKeyValueParam *>(&response.data[offset]);
    if (pair->key == kKeyPclDataType && pair->length >= 1U) {
      return pair->value[0];
    }
    offset = static_cast<uint16_t>(offset + sizeof(uint16_t) * 2U + pair->length);
  }
  return -1;
}

void onPointTypeSet(
  livox_status status, uint32_t, LivoxLidarAsyncControlResponse * response, void * client_data)
{
  auto * impl = static_cast<Impl *>(client_data);
  if (response == nullptr) {
    impl->say("livox: setting the point type got no response, status " + std::to_string(status));
    return;
  }
  impl->say(
    "livox: point type set to spherical, status " + std::to_string(status) + ", ret_code " +
    std::to_string(response->ret_code));
}

void onInternalInfo(
  livox_status status, uint32_t handle, LivoxLidarDiagInternalInfoResponse * response,
  void * client_data)
{
  auto * impl = static_cast<Impl *>(client_data);
  if (status != kLivoxLidarStatusSuccess || response == nullptr) {
    impl->say("livox: could not read the lidar settings, nothing was changed");
    return;
  }
  const int point_type = pointTypeOf(*response);
  if (point_type == static_cast<int>(kLivoxLidarSphericalCoordinateData)) {
    impl->say("livox: the lidar already sends spherical points");
    return;
  }
  if (point_type < 0) {
    impl->say("livox: the lidar did not report its point type, nothing was changed");
    return;
  }
  impl->say("livox: point type is " + std::to_string(point_type) + ", asking for spherical (3)");
  SetLivoxLidarPclDataType(handle, kLivoxLidarSphericalCoordinateData, onPointTypeSet, impl);
}

void onLidarInfoChange(const uint32_t handle, const LivoxLidarInfo * info, void * client_data)
{
  auto * impl = static_cast<Impl *>(client_data);
  if (info == nullptr) {
    return;
  }
  impl->say(
    std::string("livox: detected lidar ") + std::string(info->sn, strnlen(info->sn, 16U)) + " at " +
    std::string(info->lidar_ip, strnlen(info->lidar_ip, 16U)));
  if (!impl->configure_point_type) {
    return;
  }
  bool expected = false;
  if (!impl->detection_handled.compare_exchange_strong(expected, true)) {
    return;
  }
  QueryLivoxLidarInternalInfo(handle, onInternalInfo, impl);
}

}  // namespace

LivoxSdkSampleSource::LivoxSdkSampleSource(
  std::string config_path, Sink sink, bool configure_point_type, Log log)
: _impl(std::make_unique<detail::SdkSourceState>())
{
  if (config_path.empty()) {
    throw std::invalid_argument("LivoxSdkSampleSource needs a config file path");
  }
  if (!sink) {
    throw std::invalid_argument("LivoxSdkSampleSource needs a sink");
  }
  _impl->config_path = std::move(config_path);
  _impl->sink = std::move(sink);
  _impl->configure_point_type = configure_point_type;
  _impl->log = std::move(log);
}

LivoxSdkSampleSource::~LivoxSdkSampleSource() { stop(); }

void LivoxSdkSampleSource::setGyroSink(GyroSink sink)
{
  if (!_impl->started) {
    _impl->gyro_sink = std::move(sink);
  }
}

bool LivoxSdkSampleSource::start()
{
  if (_impl->started) {
    return true;
  }
  bool expected = false;
  if (!g_instance_running.compare_exchange_strong(expected, true)) {
    _impl->say("livox: another instance of the SDK source is running");
    return false;
  }
  if (!LivoxLidarSdkInit(_impl->config_path.c_str())) {
    LivoxLidarSdkUninit();
    g_instance_running = false;
    _impl->say("livox: the SDK did not start (config file or ports)");
    return false;
  }
  SetLivoxLidarPointCloudCallBack(onPointCloud, _impl.get());
  if (_impl->gyro_sink) {
    SetLivoxLidarImuDataCallback(onImu, _impl.get());
  }
  SetLivoxLidarInfoChangeCallback(onLidarInfoChange, _impl.get());
  _impl->started = true;
  return true;
}

void LivoxSdkSampleSource::stop()
{
  if (!_impl || !_impl->started) {
    return;
  }
  LivoxLidarSdkUninit();
  _impl->started = false;
  g_instance_running = false;
}

SdkSourceStats LivoxSdkSampleSource::stats() const
{
  SdkSourceStats result;
  result.packets = _impl->packets;
  result.decoded_packets = _impl->decoded_packets;
  result.rejected_packets = _impl->rejected_packets;
  result.not_spherical_packets = _impl->not_spherical_packets;
  result.unsynchronized_packets = _impl->unsynchronized_packets;
  result.continuity_breaks = _impl->continuity_breaks;
  result.dropped_rays = _impl->dropped_rays;
  result.imu_packets = _impl->imu_packets;
  result.imu_unusable = _impl->imu_unusable;
  return result;
}

}  // namespace livox_ray_source

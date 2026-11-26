#include "livox_ray_source/livox_sdk_sample_source.hpp"

#include <chrono>
#include <cstring>
#include <map>
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

struct DeviceState
{
  mutable std::mutex mutex;
  LivoxDeviceIdentity identity;
  UdpContinuityTracker tracker;
  PacketSampler sampler;
  std::atomic<bool> detection_handled{false};
  std::atomic<uint64_t> point_packets{0U};
  std::atomic<uint64_t> imu_packets{0U};
  std::atomic<uint64_t> decoded_points{0U};
  std::atomic<uint64_t> rejected_packets{0U};
  std::atomic<uint64_t> unsynchronized_packets{0U};
  std::atomic<uint64_t> continuity_breaks{0U};
  std::atomic<uint64_t> dropped_rays{0U};
  std::atomic<uint64_t> imu_unusable{0U};
  std::atomic<uint64_t> imu_accel_unusable{0U};
  std::atomic<uint8_t> last_point_time_type{0U};
  std::atomic<uint8_t> last_imu_time_type{0U};
  std::atomic<uint64_t> last_point_timestamp_ns{0U};
  std::atomic<uint64_t> last_imu_timestamp_ns{0U};
  std::atomic<uint64_t> last_point_host_ns{0U};
  std::atomic<uint64_t> last_imu_host_ns{0U};
};

struct SdkSourceState
{
  std::string config_path;
  LivoxSdkSampleSource::Sink sink;
  LivoxSdkSampleSource::Log log;
  LivoxSdkSampleSource::GyroSink gyro_sink;
  LivoxSdkSampleSource::ImuSink imu_sink;
  LivoxSdkSampleSource::PointSink point_sink;
  bool configure_point_type{true};
  bool started{false};
  mutable std::mutex devices_mutex;
  std::map<uint32_t, std::shared_ptr<DeviceState>> devices;
  LivoxDeviceRegistry registry;

  std::atomic<uint64_t> packets{0U};
  std::atomic<uint64_t> decoded_packets{0U};
  std::atomic<uint64_t> rejected_packets{0U};
  std::atomic<uint64_t> not_spherical_packets{0U};
  std::atomic<uint64_t> unsynchronized_packets{0U};
  std::atomic<uint64_t> continuity_breaks{0U};
  std::atomic<uint64_t> dropped_rays{0U};
  std::atomic<uint64_t> point_metadata_samples{0U};
  std::atomic<uint64_t> imu_packets{0U};
  std::atomic<uint64_t> imu_unusable{0U};
  std::atomic<uint64_t> imu_accel_unusable{0U};
  std::atomic<uint64_t> device_events{0U};
  std::atomic<uint64_t> non_primary_samples{0U};

  std::shared_ptr<DeviceState> device(uint32_t handle)
  {
    const std::lock_guard<std::mutex> lock(devices_mutex);
    auto & state = devices[handle];
    if (!state) {
      state = std::make_shared<DeviceState>();
      state->identity.handle = handle;
    }
    return state;
  }

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

uint64_t steadyNowNs()
{
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::steady_clock::now().time_since_epoch())
                                 .count());
}

// Diagnostics must stop calling a disconnected sensor "connected". This matches the approved
// 0.5 s stale-observation baseline (PERF-04); it is a health indication, not a new safety gate.
constexpr uint64_t kLivoxHealthStaleAfterNs{500'000'000U};

LivoxDeviceIdentity identityOf(const std::shared_ptr<detail::DeviceState> & state)
{
  const std::lock_guard<std::mutex> lock(state->mutex);
  return state->identity;
}

void onPointCloud(
  uint32_t handle, const uint8_t, LivoxLidarEthernetPacket * packet, void * client_data)
{
  if (packet == nullptr || client_data == nullptr) {
    return;
  }
  auto * impl = static_cast<Impl *>(client_data);
  const auto state = impl->device(handle);
  ++impl->packets;
  ++state->point_packets;
  // `length` is the size of the whole UDP payload (checked on a real Mid-360S: it equals the
  // datagram size for every packet).
  const DecodedPacket decoded =
    decodeSphericalPacket(reinterpret_cast<const uint8_t *>(packet), packet->length);
  if (decoded.status == DecodeStatus::kNotSpherical) {
    ++impl->not_spherical_packets;
    ++state->rejected_packets;
    return;
  }
  if (decoded.status != DecodeStatus::kOk) {
    ++impl->rejected_packets;
    ++state->rejected_packets;
    return;
  }
  ++impl->decoded_packets;
  state->decoded_points += decoded.rays.size();
  impl->point_metadata_samples += decoded.point_metadata.size();
  if (decoded.time_type != 1U) {
    ++impl->unsynchronized_packets;
    ++state->unsynchronized_packets;
  }
  state->last_point_time_type = decoded.time_type;
  state->last_point_timestamp_ns = decoded.rays.empty() ? 0U : decoded.rays.front().timestamp_ns;
  state->last_point_host_ns = steadyNowNs();

  std::vector<StreamSample> samples;
  std::vector<PointMetadata> metadata;
  samples.reserve(decoded.rays.size());
  metadata.reserve(decoded.rays.size());
  bool is_break = false;
  {
    const std::lock_guard<std::mutex> lock(state->mutex);
    is_break = state->tracker.update(decoded.udp_cnt);
    if (is_break) {
      ++impl->continuity_breaks;
      ++state->continuity_breaks;
    }
    const std::size_t dropped = state->sampler.append(decoded, is_break, samples, &metadata);
    impl->dropped_rays += dropped;
    state->dropped_rays += dropped;
  }

  const LivoxDeviceIdentity identity = identityOf(state);
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    const PointMetadata point_metadata =
      index < metadata.size() ? metadata[index] : PointMetadata{};
    if (impl->point_sink) {
      impl->point_sink(LivoxPointSample{identity, samples[index], point_metadata});
    }
    if (impl->registry.accepts(handle)) {
      impl->sink(samples[index]);
    }
    else {
      ++impl->non_primary_samples;
    }
  }
}

void onImu(
  const uint32_t handle, const uint8_t, LivoxLidarEthernetPacket * packet, void * client_data)
{
  if (packet == nullptr || client_data == nullptr) {
    return;
  }
  auto * impl = static_cast<Impl *>(client_data);
  const auto state = impl->device(handle);
  ++impl->imu_packets;
  ++state->imu_packets;
  const DecodedImuPacket decoded =
    decodeImuPacket(reinterpret_cast<const uint8_t *>(packet), packet->length);
  if (!decoded.gyro.isValid()) {
    ++impl->imu_unusable;
    ++state->imu_unusable;
    return;
  }
  state->last_imu_time_type = decoded.time_type;
  state->last_imu_timestamp_ns = decoded.gyro.timestamp_ns;
  state->last_imu_host_ns = steadyNowNs();
  if (!decoded.accelerationIsValid()) {
    ++impl->imu_accel_unusable;
    ++state->imu_accel_unusable;
  }
  // The gyro feeds the single safety leveler, so it must follow the same primary-device
  // selection as the safety point stream. Complete IMU telemetry remains all-device below.
  if (impl->gyro_sink && impl->registry.accepts(handle)) {
    impl->gyro_sink(decoded.gyro);
  }
  if (impl->imu_sink && decoded.accelerationIsValid()) {
    impl->imu_sink(LivoxImuSample{identityOf(state), decoded.toImuSample()});
  }
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

void onWorkModeSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  auto * impl = static_cast<Impl *>(client_data);
  if (impl == nullptr) {
    return;
  }
  const uint32_t ret_code = response == nullptr ? 0xFFFFFFFFU : response->ret_code;
  if (status == kLivoxLidarStatusSuccess && ret_code == 0U) {
    impl->say("livox: lidar " + std::to_string(handle) + " is in normal work mode");
  }
  else {
    impl->say(
      "livox: could not set lidar " + std::to_string(handle) +
      " to normal work mode, status " + std::to_string(status) + ", ret_code " +
      std::to_string(ret_code));
  }
}

void onImuEnable(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  auto * impl = static_cast<Impl *>(client_data);
  if (impl == nullptr) {
    return;
  }
  const uint32_t ret_code = response == nullptr ? 0xFFFFFFFFU : response->ret_code;
  if (status == kLivoxLidarStatusSuccess && ret_code == 0U) {
    impl->say("livox: IMU data enabled for lidar " + std::to_string(handle));
  }
  else {
    impl->say(
      "livox: could not enable IMU data for lidar " + std::to_string(handle) + ", status " +
      std::to_string(status) + ", ret_code " + std::to_string(ret_code));
  }
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
  const auto state = impl->device(handle);
  LivoxDeviceIdentity identity;
  identity.handle = handle;
  identity.device_type = info->dev_type;
  identity.serial = std::string(info->sn, strnlen(info->sn, 16U));
  identity.lidar_ip = std::string(info->lidar_ip, strnlen(info->lidar_ip, 16U));
  {
    const std::lock_guard<std::mutex> lock(state->mutex);
    state->identity = identity;
  }
  impl->registry.observe(identity);
  ++impl->device_events;
  impl->say(std::string("livox: detected lidar ") + identity.serial + " at " + identity.lidar_ip);
  bool expected = false;
  // The lifecycle commands below are idempotent and are repeated on a reconnect. The point type
  // query/write is the only command kept to one request per detection epoch.
  if (impl->configure_point_type && state->detection_handled.compare_exchange_strong(expected, true)) {
    QueryLivoxLidarInternalInfo(handle, onInternalInfo, impl);
  }

  // The SDK callback is delivered when a device is detected, but the reference driver still
  // explicitly starts normal sampling and enables the optional IMU stream. Do the same here;
  // these commands are safe to repeat after a reconnect.
  const livox_status work_mode_status =
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, onWorkModeSet, impl);
  if (work_mode_status != kLivoxLidarStatusSuccess) {
    impl->say(
      "livox: normal work-mode request failed immediately, status " +
      std::to_string(work_mode_status));
  }
  if (impl->gyro_sink || impl->imu_sink) {
    const livox_status imu_status = EnableLivoxLidarImuData(handle, onImuEnable, impl);
    if (imu_status != kLivoxLidarStatusSuccess) {
      impl->say(
        "livox: IMU enable request failed immediately, status " +
        std::to_string(imu_status));
    }
  }
}

void onLidarInfo(const uint32_t handle, const uint8_t, const char * info, void * client_data)
{
  auto * impl = static_cast<Impl *>(client_data);
  if (info != nullptr) {
    impl->say("livox: device push info handle " + std::to_string(handle) + ": " + info);
  }
}

}  // namespace

LivoxSdkSampleSource::LivoxSdkSampleSource(
  std::string config_path, Sink sink, bool configure_point_type, Log log,
  std::string primary_lidar_serial)
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
  _impl->registry.setPrimarySerial(std::move(primary_lidar_serial));
}

LivoxSdkSampleSource::~LivoxSdkSampleSource() { stop(); }

void LivoxSdkSampleSource::setGyroSink(GyroSink sink)
{
  if (!_impl->started) {
    _impl->gyro_sink = std::move(sink);
  }
}

void LivoxSdkSampleSource::setImuSink(ImuSink sink)
{
  if (!_impl->started) {
    _impl->imu_sink = std::move(sink);
  }
}

void LivoxSdkSampleSource::setPointSink(PointSink sink)
{
  if (!_impl->started) {
    _impl->point_sink = std::move(sink);
  }
}

void LivoxSdkSampleSource::setPrimaryLidarSerial(std::string serial)
{
  if (!_impl->started) {
    _impl->registry.setPrimarySerial(std::move(serial));
  }
}

bool LivoxSdkSampleSource::start()
{
  if (_impl->started) {
    return true;
  }
  {
    const std::lock_guard<std::mutex> lock(_impl->devices_mutex);
    _impl->devices.clear();
  }
  _impl->registry.clear();
  _impl->packets = 0U;
  _impl->decoded_packets = 0U;
  _impl->rejected_packets = 0U;
  _impl->not_spherical_packets = 0U;
  _impl->unsynchronized_packets = 0U;
  _impl->continuity_breaks = 0U;
  _impl->dropped_rays = 0U;
  _impl->point_metadata_samples = 0U;
  _impl->imu_packets = 0U;
  _impl->imu_unusable = 0U;
  _impl->imu_accel_unusable = 0U;
  _impl->device_events = 0U;
  _impl->non_primary_samples = 0U;
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
  if (_impl->gyro_sink || _impl->imu_sink) {
    SetLivoxLidarImuDataCallback(onImu, _impl.get());
  }
  SetLivoxLidarInfoChangeCallback(onLidarInfoChange, _impl.get());
  SetLivoxLidarInfoCallback(onLidarInfo, _impl.get());
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
  result.point_metadata_samples = _impl->point_metadata_samples;
  result.imu_packets = _impl->imu_packets;
  result.imu_unusable = _impl->imu_unusable;
  result.imu_accel_unusable = _impl->imu_accel_unusable;
  result.device_events = _impl->device_events;
  result.non_primary_samples = _impl->non_primary_samples;
  return result;
}

std::vector<LivoxDeviceHealth> LivoxSdkSampleSource::health() const
{
  std::vector<std::shared_ptr<detail::DeviceState>> devices;
  {
    const std::lock_guard<std::mutex> lock(_impl->devices_mutex);
    devices.reserve(_impl->devices.size());
    for (const auto & [handle, state] : _impl->devices) {
      static_cast<void>(handle);
      devices.push_back(state);
    }
  }

  const uint64_t now_ns = steadyNowNs();
  std::vector<LivoxDeviceHealth> result;
  result.reserve(devices.size());
  for (const auto & state : devices) {
    LivoxDeviceHealth snapshot;
    snapshot.device = identityOf(state);
    snapshot.state = LivoxHealthState::kUnknown;
    snapshot.point_packets = state->point_packets.load();
    snapshot.imu_packets = state->imu_packets.load();
    snapshot.decoded_points = state->decoded_points.load();
    snapshot.rejected_packets = state->rejected_packets.load();
    snapshot.unsynchronized_packets = state->unsynchronized_packets.load();
    snapshot.continuity_breaks = state->continuity_breaks.load();
    snapshot.dropped_rays = state->dropped_rays.load();
    snapshot.imu_unusable = state->imu_unusable.load();
    snapshot.imu_accel_unusable = state->imu_accel_unusable.load();
    snapshot.last_point_time_type = state->last_point_time_type.load();
    snapshot.last_imu_time_type = state->last_imu_time_type.load();
    snapshot.last_point_timestamp_ns = state->last_point_timestamp_ns.load();
    snapshot.last_imu_timestamp_ns = state->last_imu_timestamp_ns.load();
    const uint64_t point_host_ns = state->last_point_host_ns.load();
    const uint64_t imu_host_ns = state->last_imu_host_ns.load();
    snapshot.last_point_age_ms =
      point_host_ns == 0U || now_ns < point_host_ns ? 0U : (now_ns - point_host_ns) / 1'000'000U;
    snapshot.last_imu_age_ms =
      imu_host_ns == 0U || now_ns < imu_host_ns ? 0U : (now_ns - imu_host_ns) / 1'000'000U;
    const bool imu_expected = static_cast<bool>(_impl->gyro_sink) || static_cast<bool>(_impl->imu_sink);
    const bool point_seen = point_host_ns != 0U;
    const bool imu_seen = imu_host_ns != 0U;
    const bool point_stale =
      !point_seen || (now_ns >= point_host_ns && now_ns - point_host_ns > kLivoxHealthStaleAfterNs);
    const bool imu_stale = imu_expected &&
      (!imu_seen || (now_ns >= imu_host_ns && now_ns - imu_host_ns > kLivoxHealthStaleAfterNs));
    if (!point_seen && (!imu_expected || !imu_seen)) {
      snapshot.state = LivoxHealthState::kUnknown;
    }
    else if (point_stale || imu_stale) {
      snapshot.state = LivoxHealthState::kStale;
    }
    else {
      snapshot.state = LivoxHealthState::kConnected;
    }
    result.push_back(snapshot);
  }
  return result;
}

std::optional<LivoxDeviceIdentity> LivoxSdkSampleSource::primaryLidar() const
{
  return _impl->registry.primaryDevice();
}

bool LivoxSdkSampleSource::multipleLidarFault() const
{
  return _impl->registry.multipleDeviceFault();
}

}  // namespace livox_ray_source

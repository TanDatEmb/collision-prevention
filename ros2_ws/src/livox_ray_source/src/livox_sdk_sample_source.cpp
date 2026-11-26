#include "livox_ray_source/livox_sdk_sample_source.hpp"

#include <array>
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
  std::mutex configuration_mutex;
  std::atomic<LivoxConfigurationState> configuration{LivoxConfigurationState::kUnknown};
  std::atomic<bool> safety_ready{false};
  uint32_t configuration_pending{0U};
  std::array<uint8_t, 9U> configuration_attempts{};
  std::string configuration_error;
  std::atomic<uint64_t> point_packets{0U};
  std::atomic<uint64_t> imu_packets{0U};
  std::atomic<uint64_t> decoded_points{0U};
  std::atomic<uint64_t> rejected_packets{0U};
  std::atomic<uint64_t> unsynchronized_packets{0U};
  std::atomic<uint64_t> continuity_breaks{0U};
  std::atomic<uint64_t> dropped_rays{0U};
  std::atomic<uint64_t> imu_unusable{0U};
  std::atomic<uint64_t> imu_accel_unusable{0U};
  std::atomic<uint64_t> unsupported_packets{0U};
  std::atomic<uint64_t> wrong_data_type_packets{0U};
  std::atomic<uint64_t> configuration_retries{0U};
  std::atomic<uint8_t> last_point_time_type{0U};
  std::atomic<uint8_t> last_point_data_type{0U};
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
  std::atomic<uint64_t> unsupported_packets{0U};
  std::atomic<uint64_t> wrong_data_type_packets{0U};
  std::atomic<uint64_t> configuration_retries{0U};
  std::atomic<uint64_t> configuration_failures{0U};
  Mid360SConfig configuration;

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

constexpr uint8_t kLivoxMid360SDeviceType{35U};
constexpr uint32_t kMaxConfigurationAttempts{3U};

enum ConfigurationBit : uint32_t
{
  kConfigurePointType = 1U << 0U,
  kConfigureScanPattern = 1U << 1U,
  kConfigureBlindSpot = 1U << 2U,
  kConfigureDualEmit = 1U << 3U,
  kConfigurePpsSync = 1U << 4U,
  kConfigureImuRange = 1U << 5U,
  kConfigureInstallAttitude = 1U << 6U,
  kConfigureWorkMode = 1U << 7U,
  kConfigureImu = 1U << 8U,
};

bool isMid360S(uint8_t device_type) { return device_type == kLivoxMid360SDeviceType; }

uint8_t pointTypeValue(Mid360SPointDataType value)
{
  switch (value) {
    case Mid360SPointDataType::kSpherical:
      return kLivoxLidarSphericalCoordinateData;
    case Mid360SPointDataType::kCartesianHigh:
      return kLivoxLidarCartesianCoordinateHighData;
    case Mid360SPointDataType::kCartesianLow:
      return kLivoxLidarCartesianCoordinateLowData;
  }
  return kLivoxLidarSphericalCoordinateData;
}

LivoxLidarScanPattern scanPatternValue(Mid360SScanPattern value)
{
  switch (value) {
    case Mid360SScanPattern::kNonRepetitive:
      return kLivoxLidarScanPatternNoneRepetive;
    case Mid360SScanPattern::kRepetitive:
      return kLivoxLidarScanPatternRepetive;
    case Mid360SScanPattern::kRepetitiveLowFrameRate:
      return kLivoxLidarScanPatternRepetiveLowFrameRate;
  }
  return kLivoxLidarScanPatternNoneRepetive;
}

LivoxLidarPpsSyncMode ppsSyncValue(Mid360SPpsSyncMode value)
{
  return value == Mid360SPpsSyncMode::kSpecial ? kLivoxPpsSyncSpec : kLivoxPpsSyncNormal;
}

LivoxLidarImuOutRate imuRateValue(Mid360SImuRate value)
{
  switch (value) {
    case Mid360SImuRate::k200Hz:
      return kLivoxLidarImuOutRate200Hz;
    case Mid360SImuRate::k500Hz:
      return kLivoxLidarImuOutRate500Hz;
    case Mid360SImuRate::k100Hz:
      return kLivoxLidarImuOutRate100Hz;
    case Mid360SImuRate::k50Hz:
      return kLivoxLidarImuOutRate50Hz;
  }
  return kLivoxLidarImuOutRate200Hz;
}

LivoxLidarAccelRange accelRangeValue(Mid360SAccelRange value)
{
  switch (value) {
    case Mid360SAccelRange::k4G:
      return kLivoxLidarAccelRange4G;
    case Mid360SAccelRange::k8G:
      return kLivoxLidarAccelRange8G;
    case Mid360SAccelRange::k16G:
      return kLivoxLidarAccelRange16G;
    case Mid360SAccelRange::k32G:
      return kLivoxLidarAccelRange32G;
  }
  return kLivoxLidarAccelRange4G;
}

LivoxLidarGyroRange gyroRangeValue(Mid360SGyroRange value)
{
  switch (value) {
    case Mid360SGyroRange::k2000Dps:
      return kLivoxLidarGyroRange2000Dps;
    case Mid360SGyroRange::k1000Dps:
      return kLivoxLidarGyroRange1000Dps;
    case Mid360SGyroRange::k500Dps:
      return kLivoxLidarGyroRange500Dps;
    case Mid360SGyroRange::k250Dps:
      return kLivoxLidarGyroRange250Dps;
    case Mid360SGyroRange::k125Dps:
      return kLivoxLidarGyroRange125Dps;
    case Mid360SGyroRange::k62_5Dps:
      return kLivoxLidarGyroRange62_5Dps;
    case Mid360SGyroRange::k31_25Dps:
      return kLivoxLidarGyroRange31_25Dps;
    case Mid360SGyroRange::k15_625Dps:
      return kLivoxLidarGyroRange15_625Dps;
  }
  return kLivoxLidarGyroRange2000Dps;
}

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
  uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket * packet, void * client_data)
{
  if (packet == nullptr || client_data == nullptr) {
    return;
  }
  auto * impl = static_cast<Impl *>(client_data);
  const auto state = impl->device(handle);
  ++impl->packets;
  ++state->point_packets;
  if (!isMid360S(dev_type)) {
    ++impl->unsupported_packets;
    ++state->unsupported_packets;
    state->configuration = LivoxConfigurationState::kUnsupported;
    state->safety_ready = false;
    return;
  }
  // `length` is the size of the whole UDP payload (checked on a real Mid-360S: it equals the
  // datagram size for every packet).
  const DecodedPacket decoded =
    decodePointPacket(reinterpret_cast<const uint8_t *>(packet), packet->length);
  state->last_point_data_type = decoded.data_type;
  if (decoded.status == DecodeStatus::kNotSpherical) {
    ++impl->not_spherical_packets;
    ++impl->unsupported_packets;
    ++state->rejected_packets;
    state->safety_ready = false;
    {
      const std::lock_guard<std::mutex> lock(state->configuration_mutex);
      state->configuration = LivoxConfigurationState::kFailed;
      state->configuration_pending = 0U;
      state->configuration_error = "unsupported point data type";
    }
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
  state->last_point_data_type = decoded.data_type;
  state->last_point_timestamp_ns = decoded.rays.empty() ? 0U : decoded.rays.front().timestamp_ns;
  state->last_point_host_ns = steadyNowNs();

  const bool configured_type_matches =
    !impl->configure_point_type ||
    decoded.data_type == pointTypeValue(impl->configuration.point_data_type);
  if (!configured_type_matches) {
    ++impl->wrong_data_type_packets;
    ++state->wrong_data_type_packets;
  }

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
    if (impl->registry.accepts(handle) && state->safety_ready.load() && configured_type_matches) {
      impl->sink(samples[index]);
    }
    else {
      ++impl->non_primary_samples;
    }
  }
}

void onImu(
  const uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket * packet,
  void * client_data)
{
  if (packet == nullptr || client_data == nullptr) {
    return;
  }
  auto * impl = static_cast<Impl *>(client_data);
  const auto state = impl->device(handle);
  ++impl->imu_packets;
  ++state->imu_packets;
  if (!isMid360S(dev_type)) {
    ++impl->unsupported_packets;
    ++state->unsupported_packets;
    state->configuration = LivoxConfigurationState::kUnsupported;
    state->safety_ready = false;
    return;
  }
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

std::shared_ptr<detail::DeviceState> deviceState(Impl * impl, uint32_t handle)
{
  return impl == nullptr ? nullptr : impl->device(handle);
}

std::size_t configurationAttemptIndex(uint32_t bit)
{
  switch (bit) {
    case kConfigurePointType:
      return 0U;
    case kConfigureScanPattern:
      return 1U;
    case kConfigureBlindSpot:
      return 2U;
    case kConfigureDualEmit:
      return 3U;
    case kConfigurePpsSync:
      return 4U;
    case kConfigureImuRange:
      return 5U;
    case kConfigureInstallAttitude:
      return 6U;
    case kConfigureWorkMode:
      return 7U;
    case kConfigureImu:
      return 8U;
    default:
      return 0U;
  }
}

bool beginConfigurationAttempt(
  const std::shared_ptr<detail::DeviceState> & state, uint32_t bit, Impl * impl, uint32_t handle)
{
  if (!state) {
    return false;
  }
  const std::lock_guard<std::mutex> lock(state->configuration_mutex);
  if ((state->configuration_pending & bit) == 0U) {
    return false;
  }
  auto & attempts = state->configuration_attempts[configurationAttemptIndex(bit)];
  if (attempts >= kMaxConfigurationAttempts) {
    return false;
  }
  ++attempts;
  if (attempts > 1U) {
    ++state->configuration_retries;
    if (impl != nullptr) {
      ++impl->configuration_retries;
    }
  }
  static_cast<void>(handle);
  return true;
}

bool canRetryConfiguration(const std::shared_ptr<detail::DeviceState> & state, uint32_t bit)
{
  if (!state) {
    return false;
  }
  const std::lock_guard<std::mutex> lock(state->configuration_mutex);
  return state->configuration_attempts[configurationAttemptIndex(bit)] < kMaxConfigurationAttempts;
}

void failConfiguration(
  const std::shared_ptr<detail::DeviceState> & state, uint32_t bit, Impl * impl,
  const std::string & reason)
{
  if (!state) {
    return;
  }
  bool report = false;
  {
    const std::lock_guard<std::mutex> lock(state->configuration_mutex);
    if ((state->configuration_pending & bit) == 0U) {
      return;
    }
    state->configuration_pending = 0U;
    state->configuration = LivoxConfigurationState::kFailed;
    state->safety_ready = false;
    state->configuration_error = reason;
    if (state->configuration_attempts[configurationAttemptIndex(bit)] > 0U) {
      report = true;
      state->configuration_attempts[configurationAttemptIndex(bit)] = 0U;
    }
  }
  if (report && impl != nullptr) {
    ++impl->configuration_failures;
  }
}

void completeConfiguration(
  const std::shared_ptr<detail::DeviceState> & state, uint32_t bit, Impl * impl, uint32_t handle)
{
  if (!state) {
    return;
  }
  bool ready = false;
  {
    const std::lock_guard<std::mutex> lock(state->configuration_mutex);
    if ((state->configuration_pending & bit) == 0U) {
      return;
    }
    state->configuration_pending &= ~bit;
    ready = state->configuration_pending == 0U;
    if (ready) {
      state->configuration = LivoxConfigurationState::kReady;
      state->safety_ready = true;
      state->configuration_error.clear();
    }
  }
  if (ready && impl != nullptr) {
    impl->say("livox: Mid-360S " + std::to_string(handle) + " configuration is ready");
  }
}

bool responseOk(livox_status status, const LivoxLidarAsyncControlResponse * response)
{
  return status == kLivoxLidarStatusSuccess && response != nullptr && response->ret_code == 0U;
}

void onPointTypeSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data);
void onScanPatternSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data);
void onBlindSpotSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data);
void onDualEmitSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data);
void onPpsSyncSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data);
void onImuRangeSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data);
void onInstallAttitudeSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data);
void onWorkModeSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data);
void onImuEnable(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data);

void sendPointType(Impl * impl, uint32_t handle)
{
  const auto state = deviceState(impl, handle);
  if (!beginConfigurationAttempt(state, kConfigurePointType, impl, handle)) {
    return;
  }
  const auto status = SetLivoxLidarPclDataType(
    handle,
    static_cast<LivoxLidarPointDataType>(pointTypeValue(impl->configuration.point_data_type)),
    onPointTypeSet, impl);
  if (status != kLivoxLidarStatusSuccess) {
    if (status == kLivoxLidarStatusTimeout && canRetryConfiguration(state, kConfigurePointType)) {
      sendPointType(impl, handle);
    }
    else {
      failConfiguration(
        state, kConfigurePointType, impl, "point type command failed: " + std::to_string(status));
    }
  }
}

void sendScanPattern(Impl * impl, uint32_t handle)
{
  const auto state = deviceState(impl, handle);
  if (
    !beginConfigurationAttempt(state, kConfigureScanPattern, impl, handle) ||
    !impl->configuration.scan_pattern.has_value()) {
    return;
  }
  const auto status = SetLivoxLidarScanPattern(
    handle, scanPatternValue(*impl->configuration.scan_pattern), onScanPatternSet, impl);
  if (status != kLivoxLidarStatusSuccess) {
    if (status == kLivoxLidarStatusTimeout && canRetryConfiguration(state, kConfigureScanPattern)) {
      sendScanPattern(impl, handle);
    }
    else {
      failConfiguration(
        state, kConfigureScanPattern, impl,
        "scan pattern command failed: " + std::to_string(status));
    }
  }
}

void sendBlindSpot(Impl * impl, uint32_t handle)
{
  const auto state = deviceState(impl, handle);
  if (
    !beginConfigurationAttempt(state, kConfigureBlindSpot, impl, handle) ||
    !impl->configuration.blind_spot_mm.has_value()) {
    return;
  }
  const auto status =
    SetLivoxLidarBlindSpot(handle, *impl->configuration.blind_spot_mm, onBlindSpotSet, impl);
  if (status != kLivoxLidarStatusSuccess) {
    if (status == kLivoxLidarStatusTimeout && canRetryConfiguration(state, kConfigureBlindSpot)) {
      sendBlindSpot(impl, handle);
    }
    else {
      failConfiguration(
        state, kConfigureBlindSpot, impl, "blind spot command failed: " + std::to_string(status));
    }
  }
}

void sendDualEmit(Impl * impl, uint32_t handle)
{
  const auto state = deviceState(impl, handle);
  if (
    !beginConfigurationAttempt(state, kConfigureDualEmit, impl, handle) ||
    !impl->configuration.dual_emit.has_value()) {
    return;
  }
  const auto status =
    SetLivoxLidarDualEmit(handle, *impl->configuration.dual_emit, onDualEmitSet, impl);
  if (status != kLivoxLidarStatusSuccess) {
    if (status == kLivoxLidarStatusTimeout && canRetryConfiguration(state, kConfigureDualEmit)) {
      sendDualEmit(impl, handle);
    }
    else {
      failConfiguration(
        state, kConfigureDualEmit, impl, "dual emit command failed: " + std::to_string(status));
    }
  }
}

void sendPpsSync(Impl * impl, uint32_t handle)
{
  const auto state = deviceState(impl, handle);
  if (
    !beginConfigurationAttempt(state, kConfigurePpsSync, impl, handle) ||
    !impl->configuration.pps_sync_mode.has_value()) {
    return;
  }
  const auto status = SetLivoxLidarPpsSyncMode(
    handle, ppsSyncValue(*impl->configuration.pps_sync_mode), onPpsSyncSet, impl);
  if (status != kLivoxLidarStatusSuccess) {
    if (status == kLivoxLidarStatusTimeout && canRetryConfiguration(state, kConfigurePpsSync)) {
      sendPpsSync(impl, handle);
    }
    else {
      failConfiguration(
        state, kConfigurePpsSync, impl, "PPS sync command failed: " + std::to_string(status));
    }
  }
}

void sendImuRange(Impl * impl, uint32_t handle)
{
  const auto state = deviceState(impl, handle);
  if (
    !beginConfigurationAttempt(state, kConfigureImuRange, impl, handle) ||
    !impl->configuration.imu_range.has_value()) {
    return;
  }
  const auto & range = *impl->configuration.imu_range;
  const auto status = SetLivoxLidarImuRange(
    handle, imuRateValue(range.output_rate), accelRangeValue(range.accel_range),
    gyroRangeValue(range.gyro_range), onImuRangeSet, impl);
  if (status != kLivoxLidarStatusSuccess) {
    if (status == kLivoxLidarStatusTimeout && canRetryConfiguration(state, kConfigureImuRange)) {
      sendImuRange(impl, handle);
    }
    else {
      failConfiguration(
        state, kConfigureImuRange, impl, "IMU range command failed: " + std::to_string(status));
    }
  }
}

void sendInstallAttitude(Impl * impl, uint32_t handle)
{
  const auto state = deviceState(impl, handle);
  if (
    !impl->configuration.install_attitude.has_value() ||
    !beginConfigurationAttempt(state, kConfigureInstallAttitude, impl, handle)) {
    return;
  }
  const auto & configured = *impl->configuration.install_attitude;
  LivoxLidarInstallAttitude attitude{configured.roll_deg, configured.pitch_deg, configured.yaw_deg,
                                     configured.x_mm,     configured.y_mm,      configured.z_mm};
  const auto status = SetLivoxLidarInstallAttitude(handle, &attitude, onInstallAttitudeSet, impl);
  if (status != kLivoxLidarStatusSuccess) {
    if (
      status == kLivoxLidarStatusTimeout &&
      canRetryConfiguration(state, kConfigureInstallAttitude)) {
      sendInstallAttitude(impl, handle);
    }
    else {
      failConfiguration(
        state, kConfigureInstallAttitude, impl,
        "install attitude command failed: " + std::to_string(status));
    }
  }
}

void sendWorkMode(Impl * impl, uint32_t handle)
{
  const auto state = deviceState(impl, handle);
  if (!beginConfigurationAttempt(state, kConfigureWorkMode, impl, handle)) {
    return;
  }
  const auto status = SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, onWorkModeSet, impl);
  if (status != kLivoxLidarStatusSuccess) {
    if (status == kLivoxLidarStatusTimeout && canRetryConfiguration(state, kConfigureWorkMode)) {
      sendWorkMode(impl, handle);
    }
    else {
      failConfiguration(
        state, kConfigureWorkMode, impl,
        "normal work-mode command failed: " + std::to_string(status));
    }
  }
}

void sendImuEnable(Impl * impl, uint32_t handle)
{
  const auto state = deviceState(impl, handle);
  if (!beginConfigurationAttempt(state, kConfigureImu, impl, handle)) {
    return;
  }
  const auto status = EnableLivoxLidarImuData(handle, onImuEnable, impl);
  if (status != kLivoxLidarStatusSuccess) {
    if (status == kLivoxLidarStatusTimeout && canRetryConfiguration(state, kConfigureImu)) {
      sendImuEnable(impl, handle);
    }
    else {
      failConfiguration(
        state, kConfigureImu, impl, "IMU enable command failed: " + std::to_string(status));
    }
  }
}

void onConfigurationResponse(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data, uint32_t bit, const char * name, void (*retry)(Impl *, uint32_t))
{
  auto * impl = static_cast<Impl *>(client_data);
  const auto state = deviceState(impl, handle);
  if (responseOk(status, response)) {
    completeConfiguration(state, bit, impl, handle);
    return;
  }
  if (status == kLivoxLidarStatusTimeout && canRetryConfiguration(state, bit)) {
    retry(impl, handle);
    return;
  }
  const uint32_t ret_code = response == nullptr ? 0xFFFFFFFFU : response->ret_code;
  failConfiguration(
    state, bit, impl,
    std::string(name) + " command failed: status " + std::to_string(status) + ", ret_code " +
      std::to_string(ret_code));
}

void onPointTypeSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  onConfigurationResponse(
    status, handle, response, client_data, kConfigurePointType, "point type", sendPointType);
}

void onScanPatternSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  onConfigurationResponse(
    status, handle, response, client_data, kConfigureScanPattern, "scan pattern", sendScanPattern);
}

void onBlindSpotSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  onConfigurationResponse(
    status, handle, response, client_data, kConfigureBlindSpot, "blind spot", sendBlindSpot);
}

void onDualEmitSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  onConfigurationResponse(
    status, handle, response, client_data, kConfigureDualEmit, "dual emit", sendDualEmit);
}

void onPpsSyncSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  onConfigurationResponse(
    status, handle, response, client_data, kConfigurePpsSync, "PPS sync", sendPpsSync);
}

void onImuRangeSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  onConfigurationResponse(
    status, handle, response, client_data, kConfigureImuRange, "IMU range", sendImuRange);
}

void onInstallAttitudeSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  onConfigurationResponse(
    status, handle, response, client_data, kConfigureInstallAttitude, "install attitude",
    sendInstallAttitude);
}

void onWorkModeSet(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  onConfigurationResponse(
    status, handle, response, client_data, kConfigureWorkMode, "normal work mode", sendWorkMode);
}

void onImuEnable(
  livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse * response,
  void * client_data)
{
  onConfigurationResponse(
    status, handle, response, client_data, kConfigureImu, "IMU enable", sendImuEnable);
}

void startConfiguration(
  Impl * impl, uint32_t handle, const std::shared_ptr<detail::DeviceState> & state)
{
  uint32_t pending = kConfigureWorkMode;
  if (impl->configure_point_type) {
    pending |= kConfigurePointType;
  }
  if (impl->configuration.scan_pattern.has_value()) {
    pending |= kConfigureScanPattern;
  }
  if (impl->configuration.blind_spot_mm.has_value()) {
    pending |= kConfigureBlindSpot;
  }
  if (impl->configuration.dual_emit.has_value()) {
    pending |= kConfigureDualEmit;
  }
  if (impl->configuration.pps_sync_mode.has_value()) {
    pending |= kConfigurePpsSync;
  }
  if (impl->configuration.imu_range.has_value()) {
    pending |= kConfigureImuRange;
  }
  if (impl->configuration.install_attitude.has_value()) {
    pending |= kConfigureInstallAttitude;
  }
  if (impl->gyro_sink || impl->imu_sink) {
    pending |= kConfigureImu;
  }
  {
    const std::lock_guard<std::mutex> lock(state->configuration_mutex);
    state->configuration_pending = pending;
    state->configuration_attempts.fill(0U);
    state->configuration_error.clear();
    state->configuration = LivoxConfigurationState::kConfiguring;
    state->safety_ready = false;
  }

  if (impl->configure_point_type) sendPointType(impl, handle);
  if (impl->configuration.scan_pattern.has_value()) sendScanPattern(impl, handle);
  if (impl->configuration.blind_spot_mm.has_value()) sendBlindSpot(impl, handle);
  if (impl->configuration.dual_emit.has_value()) sendDualEmit(impl, handle);
  if (impl->configuration.pps_sync_mode.has_value()) sendPpsSync(impl, handle);
  if (impl->configuration.imu_range.has_value()) sendImuRange(impl, handle);
  if (impl->configuration.install_attitude.has_value()) sendInstallAttitude(impl, handle);
  sendWorkMode(impl, handle);
  if (impl->gyro_sink || impl->imu_sink) sendImuEnable(impl, handle);
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
  ++impl->device_events;
  impl->say(std::string("livox: detected lidar ") + identity.serial + " at " + identity.lidar_ip);
  if (!isMid360S(identity.device_type)) {
    state->configuration = LivoxConfigurationState::kUnsupported;
    state->safety_ready = false;
    impl->say("livox: ignoring unsupported device type " + std::to_string(identity.device_type));
    return;
  }
  impl->registry.observe(identity);
  {
    const std::lock_guard<std::mutex> lock(state->mutex);
    state->tracker = UdpContinuityTracker{};
    state->sampler = PacketSampler{};
  }
  startConfiguration(impl, handle, state);
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
  try {
    _impl->configuration = loadMid360SConfig(_impl->config_path);
  }
  catch (const std::exception & error) {
    _impl->say(std::string("livox: invalid Mid-360S adapter configuration: ") + error.what());
    return false;
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
  _impl->unsupported_packets = 0U;
  _impl->wrong_data_type_packets = 0U;
  _impl->configuration_retries = 0U;
  _impl->configuration_failures = 0U;
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
  result.unsupported_packets = _impl->unsupported_packets;
  result.wrong_data_type_packets = _impl->wrong_data_type_packets;
  result.configuration_retries = _impl->configuration_retries;
  result.configuration_failures = _impl->configuration_failures;
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
    snapshot.configuration = state->configuration.load();
    snapshot.unsupported_packets = state->unsupported_packets.load();
    snapshot.wrong_data_type_packets = state->wrong_data_type_packets.load();
    snapshot.configuration_retries = state->configuration_retries.load();
    snapshot.last_point_data_type = state->last_point_data_type.load();
    {
      const std::lock_guard<std::mutex> lock(state->configuration_mutex);
      snapshot.configuration_error = state->configuration_error;
    }
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
    const bool imu_expected =
      static_cast<bool>(_impl->gyro_sink) || static_cast<bool>(_impl->imu_sink);
    const bool point_seen = point_host_ns != 0U;
    const bool imu_seen = imu_host_ns != 0U;
    const bool point_stale =
      !point_seen || (now_ns >= point_host_ns && now_ns - point_host_ns > kLivoxHealthStaleAfterNs);
    const bool imu_stale =
      imu_expected &&
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

const Mid360SConfig & LivoxSdkSampleSource::configuration() const { return _impl->configuration; }

}  // namespace livox_ray_source

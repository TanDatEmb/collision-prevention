#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "livox_ray_source/ray_source.hpp"
#include "livox_ray_source/spherical_packet.hpp"

namespace livox_ray_source
{

namespace detail
{
struct SdkSourceState;  // defined in the source file
}  // namespace detail

/// SDK-independent identity copied from LivoxLidarInfo. `handle` is the SDK session key; serial is
/// the stable operator-facing identity used to select a safety lidar.
struct LivoxDeviceIdentity
{
  uint32_t handle{0U};
  uint8_t device_type{0U};
  std::string serial;
  std::string lidar_ip;
};

/// A point with its Livox metadata and device identity. `sample.ray` remains the existing safety-map
/// contract; reflectivity and tag are telemetry only.
struct LivoxPointSample
{
  LivoxDeviceIdentity device;
  StreamSample sample;
  PointMetadata metadata;
};

/// A complete IMU sample with device identity. The leveler consumes `sample.gyro` only.
struct LivoxImuSample
{
  LivoxDeviceIdentity device;
  ImuSample sample;
};

enum class LivoxHealthState : uint8_t
{
  kUnknown,
  kConnected,
  kStale,
};

/// Snapshot for diagnostics and a live probe. Rates are measured by the adapter; no health field is
/// fabricated when the SDK does not expose it.
struct LivoxDeviceHealth
{
  LivoxDeviceIdentity device;
  LivoxHealthState state{LivoxHealthState::kUnknown};
  uint64_t point_packets{0U};
  uint64_t imu_packets{0U};
  uint64_t decoded_points{0U};
  uint64_t rejected_packets{0U};
  uint64_t unsynchronized_packets{0U};
  uint64_t continuity_breaks{0U};
  uint64_t dropped_rays{0U};
  uint64_t imu_unusable{0U};
  uint64_t imu_accel_unusable{0U};
  uint8_t last_point_time_type{0U};
  uint8_t last_imu_time_type{0U};
  uint64_t last_point_timestamp_ns{0U};
  uint64_t last_imu_timestamp_ns{0U};
  uint64_t last_point_age_ms{0U};
  uint64_t last_imu_age_ms{0U};
};

/// Selects one device for the safety stream without mixing devices. With no configured serial the
/// first device is accepted only until a second device appears; then the safety stream is rejected
/// fail-closed. Raw telemetry can still expose every device.
class LivoxDeviceRegistry
{
public:
  explicit LivoxDeviceRegistry(std::string primary_serial = {});

  void clear();
  void observe(const LivoxDeviceIdentity & device);
  void setPrimarySerial(std::string serial);
  [[nodiscard]] bool accepts(uint32_t handle) const;
  [[nodiscard]] bool multipleDeviceFault() const;
  [[nodiscard]] std::optional<LivoxDeviceIdentity> primaryDevice() const;
  [[nodiscard]] std::vector<LivoxDeviceIdentity> devices() const;

private:
  mutable std::mutex _mutex;
  std::string _primary_serial;
  std::map<uint32_t, LivoxDeviceIdentity> _devices;
  std::optional<uint32_t> _primary_handle;
  bool _multiple_device_fault{false};
};

/// Counters of the packets the SDK delivered. All are updated from the SDK thread.
struct SdkSourceStats
{
  uint64_t packets{0U};
  uint64_t decoded_packets{0U};
  uint64_t rejected_packets{0U};        ///< too short, wrong length or no points
  uint64_t not_spherical_packets{0U};   ///< the lidar still sends another point type
  uint64_t unsynchronized_packets{0U};  ///< time type is not PTP: rays carry time 0
  uint64_t continuity_breaks{0U};
  uint64_t dropped_rays{0U};            ///< rays without a usable time, not handed on
  uint64_t point_metadata_samples{0U};  ///< decoded points with reflectivity/tag fields
  uint64_t imu_packets{0U};             ///< IMU packets delivered
  uint64_t imu_unusable{
    0U};  ///< IMU packets with another layout or without PTP time, not handed on
  uint64_t imu_accel_unusable{0U};   ///< valid gyro packets with a non-finite accelerometer axis
  uint64_t device_events{0U};        ///< device identity callbacks
  uint64_t non_primary_samples{0U};  ///< rays withheld from the safety sink
};

/// Spherical samples of a Livox Mid-360(S) through the Livox SDK2, as a stream of StreamSample.
///
/// Each point packet that the SDK delivers is decoded by `decodeSphericalPacket` and its rays are
/// given to `sink`, in order, from the SDK's thread (so the sink must be thread safe). A ray without
/// a usable time (the lidar is not PTP synchronized) is not given to the sink (`PacketSampler`);
/// the next ray that is, and the first ray after a gap in `udp_cnt`, carries `continuity_break`.
///
/// When `configure_point_type` is true and the lidar sends another point type, this source asks
/// the lidar for spherical points once per detection: one read, and one write (the point data
/// type, key 0x0000) only if the type differs. It writes nothing else. Like every start of the SDK,
/// the SDK itself also sends the host IP and ports from the config file to the lidar (state, point
/// and IMU destinations). The IMU callback is set only when a gyro sink is given (setGyroSink, before
/// start); its samples with PTP time go to that sink, from the SDK's thread (ADR 0023).
///
/// The SDK has one global state, so only one instance can run at a time.
class LivoxSdkSampleSource
{
public:
  using Sink = std::function<void(const StreamSample &)>;
  using Log = std::function<void(const std::string &)>;
  using GyroSink = std::function<void(const GyroSample &)>;
  using ImuSink = std::function<void(const LivoxImuSample &)>;
  using PointSink = std::function<void(const LivoxPointSample &)>;

  /// Throws std::invalid_argument for an empty `config_path` or a null `sink`. `log` may be null.
  LivoxSdkSampleSource(
    std::string config_path, Sink sink, bool configure_point_type = true, Log log = nullptr,
    std::string primary_lidar_serial = {});
  ~LivoxSdkSampleSource();

  LivoxSdkSampleSource(const LivoxSdkSampleSource &) = delete;
  LivoxSdkSampleSource & operator=(const LivoxSdkSampleSource &) = delete;

  /// Gives the IMU gyro samples to `sink` (thread safe, called from the SDK's thread). Call before start();
  /// a null sink leaves the IMU unread.
  void setGyroSink(GyroSink sink);

  /// Gives complete IMU samples to `sink`. Acceleration stays in g until a ROS boundary converts it.
  /// Call before start(). A null sink disables complete-sample publication but does not disable GyroSink.
  void setImuSink(ImuSink sink);

  /// Gives decoded points and their raw reflectivity/tag metadata to `sink`, including qualified
  /// no-return rays. Call before start().
  void setPointSink(PointSink sink);

  /// Selects the device allowed into the safety `Sink`; raw sinks still receive all devices.
  void setPrimaryLidarSerial(std::string serial);

  /// Starts the SDK. False when the SDK refuses (bad config file, port in use) or another
  /// instance is running.
  [[nodiscard]] bool start();
  void stop();

  [[nodiscard]] SdkSourceStats stats() const;
  [[nodiscard]] std::vector<LivoxDeviceHealth> health() const;
  [[nodiscard]] std::optional<LivoxDeviceIdentity> primaryLidar() const;
  [[nodiscard]] bool multipleLidarFault() const;

private:
  std::unique_ptr<detail::SdkSourceState> _impl;
};

}  // namespace livox_ray_source

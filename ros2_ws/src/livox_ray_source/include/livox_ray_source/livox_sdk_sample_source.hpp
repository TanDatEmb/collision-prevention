#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "livox_ray_source/ray_source.hpp"
#include "livox_ray_source/spherical_packet.hpp"

namespace livox_ray_source
{

namespace detail
{
struct SdkSourceState;  // defined in the source file
}  // namespace detail

/// Counters of the packets the SDK delivered. All are updated from the SDK thread.
struct SdkSourceStats
{
  uint64_t packets{0U};
  uint64_t decoded_packets{0U};
  uint64_t rejected_packets{0U};        ///< too short, wrong length or no points
  uint64_t not_spherical_packets{0U};   ///< the lidar still sends another point type
  uint64_t unsynchronized_packets{0U};  ///< time type is not PTP: rays carry time 0
  uint64_t continuity_breaks{0U};
  uint64_t dropped_rays{0U};  ///< rays without a usable time, not handed on
  uint64_t imu_packets{0U};   ///< IMU packets delivered (counted only with a gyro sink)
  uint64_t imu_unusable{0U};  ///< IMU packets with another layout or without PTP time, not handed on
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

  /// Throws std::invalid_argument for an empty `config_path` or a null `sink`. `log` may be null.
  LivoxSdkSampleSource(
    std::string config_path, Sink sink, bool configure_point_type = true, Log log = nullptr);
  ~LivoxSdkSampleSource();

  LivoxSdkSampleSource(const LivoxSdkSampleSource &) = delete;
  LivoxSdkSampleSource & operator=(const LivoxSdkSampleSource &) = delete;

  /// Gives the IMU gyro samples to `sink` (thread safe, called from the SDK's thread). Call before start();
  /// a null sink leaves the IMU unread.
  void setGyroSink(GyroSink sink);

  /// Starts the SDK. False when the SDK refuses (bad config file, port in use) or another
  /// instance is running.
  [[nodiscard]] bool start();
  void stop();

  [[nodiscard]] SdkSourceStats stats() const;

private:
  std::unique_ptr<detail::SdkSourceState> _impl;
};

}  // namespace livox_ray_source

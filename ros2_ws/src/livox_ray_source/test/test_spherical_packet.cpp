#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <array>
#include <cstring>
#include <string>
#include <vector>

#include "livox_ray_source/frame_segmenter.hpp"
#include "livox_ray_source/spherical_packet.hpp"

using livox_ray_source::DecodedPacket;
using livox_ray_source::decodeSphericalPacket;
using livox_ray_source::DecodeStatus;
using livox_ray_source::FrameSegmenter;
using livox_ray_source::PacketSampler;
using livox_ray_source::StreamSample;
using livox_ray_source::UdpContinuityTracker;
using obstacle_sector_map::RayReturnKind;

namespace
{

constexpr float kPi = 3.14159265358979323846F;
constexpr uint64_t kPtpTimeNs = 1'795'000'000'000'000'000U;

struct Point
{
  uint32_t depth_mm;
  uint16_t theta;
  uint16_t phi;
  uint8_t reflectivity{0U};
  uint8_t tag{0U};
};

// One real packet of a Mid-360S (firmware app 35.1.1.8) in spherical mode, 2026-11-20, lidar at
// rest and not synchronized: header version 0, length 996, time_interval 4750, dot_num 96, udp_cnt
// 23052, frame_cnt 0, data_type 3, time_type 0, crc32 0, timestamp 5055226547740 ns since power on.
// 21 of its 96 points have depth 0. The values asserted below were decoded independently with a
// Python script from the same bytes.
const char * const kRealPacketHex =
  "00e4038e1260000c5a000300000000000000000000000000000000001c8efb0299040000500300005d1be73a02005903"
  "0000c31af73a0200660300002f1a033b0300490300009219063b0200610300005f1b713b04006c030000c51a7a3b0400"
  "74030000311a813b04004b0300009519803b02005c030000611bf93b050061030000c71afe3b040068030000341a003c"
  "04004b0300009719f93b020052030000631b833c050057030000c91a823c04005a030000351a7e3c0400470300009919"
  "723c020046030000651b0b3d04004e030000cb1a063d04004b030000381afc3c04004f0300009c19eb3c02003f030000"
  "671b943d050046030000cd1a893d05004f0300003a1a7b3d0500500300009f19643d030032030000691b1d3e05003d03"
  "0000cf1a0c3e0500430300003d1af93d050049030000a119dc3d04002f0300006b1ba53e050033030000d11a8f3e0500"
  "400300003f1a783e050040030000a519563e0500230300006c1b2d3f05002b030000d31a133f040031030000421af63e"
  "040040030000a819cf3e05001f0300006e1bb63f050019030000d61a973f05002e030000441a753f050036030000ac19"
  "493f0500110300006f1b3e40060019030000d81a1b40050027030000481af43f050033030000af19c13f0500e3000000"
  "711bc8400200ed000000da1a9e400100e20000004b1a72400100e5000000b3193a400200e4000000721b50410200de00"
  "0000dd1a22410100e10000004e1af1400100e6000000b619b340020000000000731bd841000000000000df1aa5410000"
  "00000000501a6f41000000000000bb192d41000000000000751b5f42000000000000e11a2842000000000000541aee41"
  "000000000000bf19a64100004e170000761be8420b00c7150000e31aab42000084140000571a6d42010089130000c419"
  "21421000f1170000771b7143100070160000e61a2f430f00000000005b1aec42000000000000c8199a42000000000000"
  "781bf943000000000000e81ab2430000000000005e1a6a43000000000000cd1914430000e4030000791b80440100dd03"
  "0000ea1a3644040000000000611ae943000000000000d2198e4300001c0400007b1b0945020099030000ed1ab9440100"
  "9e030000651a6744020069030000d71907440200df0300007b1b8f4500007f030000ef1a3c45010046030000691ae744"
  "010068030000dc1981440200000000007c1b18460000ac080000f11abf450104430300006d1a654500000a030000e219"
  "fc440102c20200007d1b9f46000000000000f31a4346000000000000711ae545000078030000e7197545000082020000"
  "7e1b2747030000000000f61ac6460000dd020000751a6346000000000000ed19f0450000";

template <typename T>
void put(std::vector<uint8_t> & bytes, std::size_t offset, T value)
{
  std::memcpy(
    bytes.data() + offset, &value, sizeof(T));  // the host is little endian, like the lidar
}

// Layout of the Livox Mid-360 point packet (protocol document 1.2.1.3.1): 36 byte header, then
// the points. Offsets: version 0, length 1, time_interval 3 (0.1 us), dot_num 5, udp_cnt 7,
// frame_cnt 9, data_type 10, time_type 11, reserved 12, crc32 24, timestamp 28 (ns).
std::vector<uint8_t> makePacket(
  const std::vector<Point> & points, uint8_t data_type = 3U, uint8_t time_type = 1U,
  uint16_t time_interval = 20U, uint16_t udp_cnt = 7U, uint64_t timestamp_ns = kPtpTimeNs)
{
  std::vector<uint8_t> bytes(36U + points.size() * 10U, 0U);
  put<uint16_t>(bytes, 1U, static_cast<uint16_t>(bytes.size()));
  put<uint16_t>(bytes, 3U, time_interval);
  put<uint16_t>(bytes, 5U, static_cast<uint16_t>(points.size()));
  put<uint16_t>(bytes, 7U, udp_cnt);
  bytes[10] = data_type;
  bytes[11] = time_type;
  put<uint64_t>(bytes, 28U, timestamp_ns);
  for (std::size_t index = 0U; index < points.size(); ++index) {
    const std::size_t at = 36U + index * 10U;
    put<uint32_t>(bytes, at, points[index].depth_mm);
    put<uint16_t>(bytes, at + 4U, points[index].theta);
    put<uint16_t>(bytes, at + 6U, points[index].phi);
    bytes[at + 8U] = points[index].reflectivity;
    bytes[at + 9U] = points[index].tag;
  }
  return bytes;
}

TEST(DecodeSphericalPacketTest, PointsBecomeRaysWithRangeInMetresAndTheVendorAngles)
{
  // theta 9000 = 90 deg from +z = elevation 0; phi 0 = along +x, phi 9000 = along +y (left).
  const auto bytes = makePacket({{2000U, 9000U, 0U}, {1000U, 9000U, 9000U}});
  const DecodedPacket packet = decodeSphericalPacket(bytes.data(), bytes.size());
  ASSERT_EQ(packet.status, DecodeStatus::kOk);
  ASSERT_EQ(packet.rays.size(), 2U);
  EXPECT_EQ(packet.rays[0].return_kind, RayReturnKind::kReturn);
  EXPECT_FLOAT_EQ(packet.rays[0].range_m, 2.0F);
  EXPECT_NEAR(packet.rays[0].azimuth_rad, 0.0F, 1e-6F);
  EXPECT_NEAR(packet.rays[0].elevation_rad, 0.0F, 1e-6F);
  EXPECT_FLOAT_EQ(packet.rays[1].range_m, 1.0F);
  EXPECT_NEAR(packet.rays[1].azimuth_rad, kPi / 2.0F, 1e-6F);
  EXPECT_FALSE(packet.rays[0].masked);
}

TEST(DecodeSphericalPacketTest, ElevationIsNinetyDegreesMinusThetaAndTheEndsAreCovered)
{
  const auto bytes = makePacket({{1000U, 0U, 0U}, {1000U, 18000U, 18000U}, {1000U, 4800U, 27000U}});
  const auto packet = decodeSphericalPacket(bytes.data(), bytes.size());
  ASSERT_EQ(packet.rays.size(), 3U);
  EXPECT_NEAR(packet.rays[0].elevation_rad, kPi / 2.0F, 1e-6F);   // straight up
  EXPECT_NEAR(packet.rays[1].elevation_rad, -kPi / 2.0F, 1e-6F);  // straight down
  EXPECT_NEAR(packet.rays[1].azimuth_rad, kPi, 1e-6F);
  // theta 48 deg: 42 deg above the horizon; phi 270 deg.
  EXPECT_NEAR(packet.rays[2].elevation_rad, 42.0F * kPi / 180.0F, 1e-5F);
  EXPECT_NEAR(packet.rays[2].azimuth_rad, 270.0F * kPi / 180.0F, 1e-5F);
}

TEST(DecodeSphericalPacketTest, ADepthOfZeroWithAValidDirectionIsAQualifiedNoReturn)
{
  const auto bytes = makePacket({{0U, 9000U, 0U}, {50U, 9000U, 0U}});
  const auto packet = decodeSphericalPacket(bytes.data(), bytes.size());
  ASSERT_EQ(packet.rays.size(), 2U);
  EXPECT_EQ(packet.rays[0].return_kind, RayReturnKind::kQualifiedNoReturn);
  EXPECT_TRUE(std::isnan(packet.rays[0].range_m));
  EXPECT_NEAR(packet.rays[0].elevation_rad, 0.0F, 1e-6F);  // the direction is kept
  // 5 cm is a real, very close return; the map decides what too close means.
  EXPECT_EQ(packet.rays[1].return_kind, RayReturnKind::kReturn);
}

TEST(DecodeSphericalPacketTest, ADepthOfZeroWithADirectionOutsideTheRangeIsInvalidNotAClearRay)
{
  const auto bytes = makePacket({{0U, 18001U, 0U}, {0U, 9000U, 36001U}});
  const auto packet = decodeSphericalPacket(bytes.data(), bytes.size());
  ASSERT_EQ(packet.rays.size(), 2U);
  EXPECT_EQ(packet.rays[0].return_kind, RayReturnKind::kInvalid);
  EXPECT_EQ(packet.rays[1].return_kind, RayReturnKind::kInvalid);
}

TEST(DecodeSphericalPacketTest, AnglesOutsideTheDocumentedRangeAreInvalid)
{
  const auto bytes =
    makePacket({{1000U, 18001U, 0U}, {1000U, 9000U, 36001U}, {1000U, 9000U, 36000U}});
  const auto packet = decodeSphericalPacket(bytes.data(), bytes.size());
  ASSERT_EQ(packet.rays.size(), 3U);
  EXPECT_EQ(packet.rays[0].return_kind, RayReturnKind::kInvalid);
  EXPECT_EQ(packet.rays[1].return_kind, RayReturnKind::kInvalid);
  EXPECT_EQ(packet.rays[2].return_kind, RayReturnKind::kReturn);  // 360.00 deg is the last value
}

TEST(DecodeSphericalPacketTest, PointTimesAreEquallySpacedOverTimeIntervalFromThePacketTime)
{
  // time_interval 4750 x 0.1 us = 475 us from the first to the last of 96 points: 5 us apart.
  std::vector<Point> points(96U, Point{1000U, 9000U, 0U});
  const auto bytes = makePacket(points, 3U, 1U, 4750U);
  const auto packet = decodeSphericalPacket(bytes.data(), bytes.size());
  ASSERT_EQ(packet.rays.size(), 96U);
  EXPECT_EQ(packet.rays[0].timestamp_ns, kPtpTimeNs);
  EXPECT_EQ(packet.rays[1].timestamp_ns, kPtpTimeNs + 5'000U);
  EXPECT_EQ(packet.rays[95].timestamp_ns, kPtpTimeNs + 475'000U);
}

TEST(DecodeSphericalPacketTest, APacketOfOnePointHasNoSpacing)
{
  const auto bytes = makePacket({{1000U, 9000U, 0U}}, 3U, 1U, 4750U);
  const auto packet = decodeSphericalPacket(bytes.data(), bytes.size());
  ASSERT_EQ(packet.rays.size(), 1U);
  EXPECT_EQ(packet.rays[0].timestamp_ns, kPtpTimeNs);
}

TEST(DecodeSphericalPacketTest, OnlyPtpTimeIsUsedAnythingElseGivesZeroSoItIsRejectedDownstream)
{
  for (uint8_t time_type : {0U, 2U, 3U}) {
    const auto bytes = makePacket({{1000U, 9000U, 0U}}, 3U, time_type);
    const auto packet = decodeSphericalPacket(bytes.data(), bytes.size());
    ASSERT_EQ(packet.status, DecodeStatus::kOk);
    EXPECT_EQ(packet.time_type, time_type);
    EXPECT_EQ(packet.rays.at(0).timestamp_ns, 0U) << "time_type " << static_cast<int>(time_type);
    EXPECT_EQ(packet.rays.at(0).return_kind, RayReturnKind::kReturn);  // direction is kept
  }
}

TEST(DecodeSphericalPacketTest, ReportsTheUdpCounterAndTheTimeType)
{
  const auto bytes = makePacket({{1000U, 9000U, 0U}}, 3U, 1U, 20U, 65535U);
  const auto packet = decodeSphericalPacket(bytes.data(), bytes.size());
  EXPECT_EQ(packet.udp_cnt, 65535U);
  EXPECT_EQ(packet.time_type, 1U);
}

TEST(DecodeSphericalPacketTest, BadPacketsAreRejectedWithTheirReason)
{
  const auto good = makePacket({{1000U, 9000U, 0U}});
  EXPECT_EQ(decodeSphericalPacket(nullptr, 0U).status, DecodeStatus::kTooShort);
  EXPECT_EQ(decodeSphericalPacket(good.data(), 35U).status, DecodeStatus::kTooShort);

  const auto cartesian = makePacket({{1000U, 9000U, 0U}}, 1U);
  EXPECT_EQ(
    decodeSphericalPacket(cartesian.data(), cartesian.size()).status, DecodeStatus::kNotSpherical);
  const auto imu = makePacket({}, 0U);
  EXPECT_EQ(decodeSphericalPacket(imu.data(), imu.size()).status, DecodeStatus::kNotSpherical);

  // The packet is shorter than dot_num says, or longer.
  EXPECT_EQ(
    decodeSphericalPacket(good.data(), good.size() - 1U).status, DecodeStatus::kLengthMismatch);
  auto longer = good;
  longer.push_back(0U);
  EXPECT_EQ(
    decodeSphericalPacket(longer.data(), longer.size()).status, DecodeStatus::kLengthMismatch);

  const auto empty = makePacket({}, 3U);
  EXPECT_EQ(decodeSphericalPacket(empty.data(), empty.size()).status, DecodeStatus::kNoPoints);
}

TEST(UdpContinuityTrackerTest, AGapDuplicateOrStepBackIsABreakAndTheCounterWraps)
{
  UdpContinuityTracker tracker;
  EXPECT_FALSE(tracker.update(10U));  // first packet: nothing is known to be missing
  EXPECT_FALSE(tracker.update(11U));
  EXPECT_TRUE(tracker.update(13U));  // 12 is missing
  EXPECT_FALSE(tracker.update(14U));
  EXPECT_TRUE(tracker.update(14U));  // duplicate
  EXPECT_TRUE(tracker.update(9U));   // backwards
  EXPECT_FALSE(tracker.update(10U));
  UdpContinuityTracker wrap;
  EXPECT_FALSE(wrap.update(65535U));
  EXPECT_FALSE(wrap.update(0U));  // 65535 + 1 wraps to 0
  EXPECT_FALSE(wrap.update(1U));
}

std::vector<uint8_t> fromHex(const std::string & hex)
{
  std::vector<uint8_t> bytes;
  for (std::size_t at = 0U; at + 1U < hex.size(); at += 2U) {
    bytes.push_back(static_cast<uint8_t>(std::stoul(hex.substr(at, 2U), nullptr, 16)));
  }
  return bytes;
}

TEST(DecodeSphericalPacketTest, ARealMid360sPacketDecodesAsTheIndependentScriptDid)
{
  const auto bytes = fromHex(kRealPacketHex);
  ASSERT_EQ(bytes.size(), 996U);
  const DecodedPacket packet = decodeSphericalPacket(bytes.data(), bytes.size());
  ASSERT_EQ(packet.status, DecodeStatus::kOk);
  ASSERT_EQ(packet.rays.size(), 96U);
  EXPECT_EQ(packet.udp_cnt, 23052U);
  EXPECT_EQ(packet.time_type, 0U);

  // First point: depth 848 mm, theta 7005 (70.05 deg from +z), phi 15079 (150.79 deg).
  EXPECT_FLOAT_EQ(packet.rays[0].range_m, 0.848F);
  EXPECT_NEAR(packet.rays[0].azimuth_rad, 150.79F * kPi / 180.0F, 1e-5F);
  EXPECT_NEAR(packet.rays[0].elevation_rad, (90.0F - 70.05F) * kPi / 180.0F, 1e-5F);

  std::size_t invalid = 0U;
  std::size_t no_returns = 0U;
  std::size_t returns = 0U;
  for (const auto & ray : packet.rays) {
    EXPECT_EQ(ray.timestamp_ns, 0U);  // not synchronized: the time is not used
    invalid += ray.return_kind == RayReturnKind::kInvalid ? 1U : 0U;
    no_returns += ray.return_kind == RayReturnKind::kQualifiedNoReturn ? 1U : 0U;
    returns += ray.return_kind == RayReturnKind::kReturn ? 1U : 0U;
  }
  EXPECT_EQ(invalid, 0U);
  EXPECT_EQ(no_returns, 21U);  // the 21 points of depth 0 of this real packet
  EXPECT_EQ(returns, 75U);
}

TEST(DecodeSphericalPacketTest, TheSameRealPacketOnPtpTimeSpreadsItsPointsOver475Microseconds)
{
  auto bytes = fromHex(kRealPacketHex);
  bytes[11] = 1U;  // time_type: PTP
  const DecodedPacket packet = decodeSphericalPacket(bytes.data(), bytes.size());
  ASSERT_EQ(packet.status, DecodeStatus::kOk);
  EXPECT_EQ(packet.rays.front().timestamp_ns, 5'055'226'547'740U);
  EXPECT_EQ(packet.rays.back().timestamp_ns, 5'055'226'547'740U + 475'000U);
}

DecodedPacket decodeThree(uint8_t time_type, uint16_t udp_cnt = 1U)
{
  const auto bytes = makePacket(
    {{1000U, 9000U, 0U}, {2000U, 9000U, 100U}, {3000U, 9000U, 200U}}, 3U, time_type, 20U, udp_cnt);
  return decodeSphericalPacket(bytes.data(), bytes.size());
}

TEST(PacketSamplerTest, RaysWithATimeAreHandedOnInOrderAndUnbroken)
{
  PacketSampler sampler;
  std::vector<StreamSample> out;
  EXPECT_EQ(sampler.append(decodeThree(1U), false, out), 0U);
  ASSERT_EQ(out.size(), 3U);
  EXPECT_FLOAT_EQ(out[0].ray.range_m, 1.0F);
  EXPECT_FLOAT_EQ(out[2].ray.range_m, 3.0F);
  EXPECT_LT(out[0].ray.timestamp_ns, out[2].ray.timestamp_ns);
  for (const auto & sample : out) {
    EXPECT_FALSE(sample.continuity_break);
  }
}

TEST(PacketSamplerTest, RaysWithoutATimeAreDroppedAndCounted)
{
  PacketSampler sampler;
  std::vector<StreamSample> out;
  EXPECT_EQ(sampler.append(decodeThree(0U), false, out), 3U);
  EXPECT_TRUE(out.empty());
}

TEST(PacketSamplerTest, AUdpGapMarksOnlyTheFirstRayOfThePacket)
{
  PacketSampler sampler;
  std::vector<StreamSample> out;
  static_cast<void>(sampler.append(decodeThree(1U), true, out));
  ASSERT_EQ(out.size(), 3U);
  EXPECT_TRUE(out[0].continuity_break);
  EXPECT_FALSE(out[1].continuity_break);
  EXPECT_FALSE(out[2].continuity_break);
}

TEST(PacketSamplerTest, ABreakOrADropBeforeTheNextUsableRayIsNotForgotten)
{
  PacketSampler sampler;
  std::vector<StreamSample> out;
  // The lidar loses its time, and a packet is also missing in the middle of that.
  static_cast<void>(sampler.append(decodeThree(0U), false, out));
  static_cast<void>(sampler.append(decodeThree(0U), true, out));
  EXPECT_TRUE(out.empty());
  // The first ray after the gap carries the break, the rest do not.
  static_cast<void>(sampler.append(decodeThree(1U), false, out));
  ASSERT_EQ(out.size(), 3U);
  EXPECT_TRUE(out[0].continuity_break);
  EXPECT_FALSE(out[1].continuity_break);
  // And it is spent once reported.
  out.clear();
  static_cast<void>(sampler.append(decodeThree(1U), false, out));
  ASSERT_EQ(out.size(), 3U);
  EXPECT_FALSE(out[0].continuity_break);
}

TEST(PacketSamplerTest, AStreamWithoutTimeBuffersNothingInTheSegmenter)
{
  // 200,000 points per second without PTP: the segmenter must not collect them (it only closes a
  // frame on a ray with a time, so it would grow without bound).
  PacketSampler sampler;
  FrameSegmenter segmenter(100'000'000U);
  std::vector<StreamSample> out;
  std::size_t forwarded = 0U;
  for (uint16_t packet = 0U; packet < 5000U; ++packet) {
    out.clear();
    static_cast<void>(sampler.append(decodeThree(0U, packet), packet % 97U == 0U, out));
    for (const auto & sample : out) {
      static_cast<void>(segmenter.push(sample));
      ++forwarded;
    }
  }
  EXPECT_EQ(forwarded, 0U);
  EXPECT_FALSE(segmenter.flush().has_value());
}

}  // namespace

namespace
{

// A Livox IMU packet: the 36 byte header with data type 0, then gyro x, y, z (rad/s) and acceleration x, y,
// z (g) as float32.
std::vector<uint8_t> makeImuPacket(
  const std::array<float, 6> & values, uint8_t time_type = 1U, uint64_t timestamp_ns = kPtpTimeNs,
  std::size_t extra_bytes = 0U)
{
  std::vector<uint8_t> bytes(36U + 24U + extra_bytes, 0U);
  put<uint16_t>(bytes, 1U, static_cast<uint16_t>(bytes.size()));
  bytes[10] = 0U;
  bytes[11] = time_type;
  put<uint64_t>(bytes, 28U, timestamp_ns);
  for (std::size_t i = 0U; i < 6U; ++i) {
    put<float>(bytes, 36U + 4U * i, values[i]);
  }
  return bytes;
}

}  // namespace

TEST(DecodeImuPacketTest, GyroRatesAndThePtpTimeAreRead)
{
  const auto bytes = makeImuPacket({0.1F, -0.2F, 0.3F, 0.0F, 0.0F, 1.0F});
  const auto packet = livox_ray_source::decodeImuPacket(bytes.data(), bytes.size());
  ASSERT_TRUE(packet.layout_ok);
  EXPECT_EQ(packet.time_type, 1U);
  ASSERT_TRUE(packet.gyro.isValid());
  EXPECT_EQ(packet.gyro.timestamp_ns, kPtpTimeNs);
  EXPECT_DOUBLE_EQ(packet.gyro.rate_flu_rad_s[0], static_cast<double>(0.1F));
  EXPECT_DOUBLE_EQ(packet.gyro.rate_flu_rad_s[1], static_cast<double>(-0.2F));
  EXPECT_DOUBLE_EQ(packet.gyro.rate_flu_rad_s[2], static_cast<double>(0.3F));
}

TEST(DecodeImuPacketTest, NoPtpTimeOrAnotherLayoutGivesNoSample)
{
  const std::array<float, 6> values{0.1F, 0.2F, 0.3F, 0.0F, 0.0F, 1.0F};
  const auto unsynchronized = makeImuPacket(values, 0U);
  auto packet = livox_ray_source::decodeImuPacket(unsynchronized.data(), unsynchronized.size());
  EXPECT_TRUE(packet.layout_ok);
  EXPECT_FALSE(packet.gyro.isValid());
  const auto zero_time = makeImuPacket(values, 1U, 0U);
  EXPECT_FALSE(livox_ray_source::decodeImuPacket(zero_time.data(), zero_time.size()).gyro.isValid());
  const auto too_long = makeImuPacket(values, 1U, kPtpTimeNs, 4U);
  packet = livox_ray_source::decodeImuPacket(too_long.data(), too_long.size());
  EXPECT_FALSE(packet.layout_ok);
  EXPECT_FALSE(packet.gyro.isValid());
  auto point_type = makeImuPacket(values);
  point_type[10] = 3U;
  EXPECT_FALSE(livox_ray_source::decodeImuPacket(point_type.data(), point_type.size()).layout_ok);
  EXPECT_FALSE(livox_ray_source::decodeImuPacket(nullptr, 60U).layout_ok);
}

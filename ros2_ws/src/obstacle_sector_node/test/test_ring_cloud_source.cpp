#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

#include "obstacle_sector_node/ring_cloud_source.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

using obstacle_sector_map::RayReturnKind;
using obstacle_sector_node::RingCloudLayout;
using obstacle_sector_node::ringCloudToFrame;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

namespace
{

constexpr double kPi{3.14159265358979323846};
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// The bench layout of the simulated multi-ring lidar (tools/scenarios/models/lidar_multi_ring/model.sdf),
// with a small step count where the test does not need the real size.
RingCloudLayout benchLayout()
{
  RingCloudLayout layout;
  layout.azimuth_count = 720U;
  layout.azimuth_min_rad = -kPi;
  layout.azimuth_step_rad = 2.0 * kPi / 720.0;
  layout.ring_count = 60U;
  layout.elevation_min_rad = -7.0 * kPi / 180.0;
  layout.elevation_step_rad = kPi / 180.0;
  layout.range_min_m = 0.1F;
  layout.range_max_m = 50.0F;
  layout.max_index_angle_error_rad = 0.002;
  return layout;
}

std::array<double, 3> indexDirection(const RingCloudLayout & layout, uint32_t ring, uint32_t step)
{
  const double elevation = layout.elevation_min_rad + ring * layout.elevation_step_rad;
  const double azimuth = layout.azimuth_min_rad + step * layout.azimuth_step_rad;
  return {
    std::cos(elevation) * std::cos(azimuth), std::cos(elevation) * std::sin(azimuth),
    std::sin(elevation)};
}

// An organised cloud like the Gazebo one: x, y, z at 0, 4, 8, intensity at 16, ring (always 0) at 24,
// point_step 32. `point(ring, step)` gives x, y, z.
template <typename PointFn>
PointCloud2 makeCloud(const RingCloudLayout & layout, PointFn point)
{
  PointCloud2 cloud;
  cloud.header.stamp.sec = 7;
  cloud.header.stamp.nanosec = 500'000'000U;
  cloud.header.frame_id = "lidar";
  cloud.width = layout.azimuth_count;
  cloud.height = layout.ring_count;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  cloud.point_step = 32U;
  cloud.row_step = cloud.point_step * cloud.width;
  const auto add_field = [&cloud](const char * name, uint32_t offset, uint8_t type) {
    PointField field;
    field.name = name;
    field.offset = offset;
    field.datatype = type;
    field.count = 1U;
    cloud.fields.push_back(field);
  };
  add_field("x", 0U, PointField::FLOAT32);
  add_field("y", 4U, PointField::FLOAT32);
  add_field("z", 8U, PointField::FLOAT32);
  add_field("intensity", 16U, PointField::FLOAT32);
  add_field("ring", 24U, PointField::UINT16);
  cloud.data.assign(static_cast<std::size_t>(cloud.row_step) * cloud.height, 0U);
  for (uint32_t ring = 0U; ring < layout.ring_count; ++ring) {
    for (uint32_t step = 0U; step < layout.azimuth_count; ++step) {
      const std::array<double, 3> xyz = point(ring, step);
      uint8_t * out = cloud.data.data() + static_cast<std::size_t>(ring) * cloud.row_step +
                      static_cast<std::size_t>(step) * cloud.point_step;
      for (std::size_t axis = 0U; axis < 3U; ++axis) {
        const float value = static_cast<float>(xyz[axis]);
        std::memcpy(out + 4U * axis, &value, sizeof(float));
      }
    }
  }
  return cloud;
}

// The point of a ray with the given range along the index direction.
std::array<double, 3> at(const RingCloudLayout & layout, uint32_t ring, uint32_t step, double range)
{
  const auto d = indexDirection(layout, ring, step);
  return {range * d[0], range * d[1], range * d[2]};
}

// A component of a direction that is exactly zero would give NaN (inf * 0). Gazebo writes no NaN (0 of 43200
// points in a live cloud), so the helper keeps a tiny non-zero component instead.
double nonZero(double component) { return component != 0.0 ? component : 1e-30; }

// A no-hit point as Gazebo writes it: range +inf times the direction (the signs of the components follow the
// direction).
std::array<double, 3> noHit(const RingCloudLayout & layout, uint32_t ring, uint32_t step)
{
  const auto d = indexDirection(layout, ring, step);
  return {kInf * nonZero(d[0]), kInf * nonZero(d[1]), kInf * nonZero(d[2])};
}

std::size_t flat(const RingCloudLayout & layout, uint32_t ring, uint32_t step)
{
  return static_cast<std::size_t>(ring) * layout.azimuth_count + step;
}

TEST(RingCloudLayoutTest, TheBenchLayoutIsValidAndEveryDefaultIsNot)
{
  EXPECT_TRUE(benchLayout().valid());
  EXPECT_FALSE(RingCloudLayout{}.valid());
  for (int broken = 0; broken < 9; ++broken) {
    auto layout = benchLayout();
    switch (broken) {
      case 0: layout.azimuth_count = 0U; break;
      case 1: layout.azimuth_step_rad = 0.0; break;
      case 2: layout.ring_count = 0U; break;
      case 3: layout.elevation_step_rad = kNaN; break;
      case 4: layout.range_min_m = 0.0F; break;
      case 5: layout.range_max_m = layout.range_min_m; break;
      case 6: layout.max_index_angle_error_rad = 0.0; break;
      case 7: layout.azimuth_min_rad = kNaN; break;
      default: layout.elevation_min_rad = kInf; break;
    }
    EXPECT_FALSE(layout.valid()) << "case " << broken;
  }
}

TEST(RingCloudToFrameTest, AnInvalidLayoutThrows)
{
  const auto layout = benchLayout();
  const auto cloud = makeCloud(layout, [&](uint32_t r, uint32_t s) { return at(layout, r, s, 3.0); });
  EXPECT_THROW((void)ringCloudToFrame(cloud, RingCloudLayout{}, true), std::invalid_argument);
}

TEST(RingCloudToFrameTest, EveryRayTakesItsAnglesFromTheIndexAndItsRangeFromThePoint)
{
  const auto layout = benchLayout();
  const auto cloud = makeCloud(
    layout, [&](uint32_t r, uint32_t s) { return at(layout, r, s, 2.0 + 0.01 * r); });
  const auto result = ringCloudToFrame(cloud, layout, true);
  ASSERT_TRUE(result.stats.shape_ok);
  ASSERT_EQ(result.frame.rays.size(), 43200U);
  EXPECT_EQ(result.stats.returns, 43200U);
  EXPECT_EQ(result.stats.index_mismatch, 0U);
  EXPECT_TRUE(result.frame.packet_continuity_valid);
  for (uint32_t ring : {0U, 17U, 59U}) {
    for (uint32_t step : {0U, 1U, 360U, 540U, 719U}) {
      const auto & ray = result.frame.rays[flat(layout, ring, step)];
      EXPECT_EQ(ray.return_kind, RayReturnKind::kReturn);
      EXPECT_NEAR(ray.range_m, 2.0 + 0.01 * ring, 1e-4);
      EXPECT_NEAR(
        ray.azimuth_rad, static_cast<float>(layout.azimuth_min_rad + step * layout.azimuth_step_rad),
        1e-6);
      EXPECT_NEAR(
        ray.elevation_rad,
        static_cast<float>(layout.elevation_min_rad + ring * layout.elevation_step_rad), 1e-6);
      EXPECT_EQ(ray.timestamp_ns, 7'500'000'000U);
      EXPECT_FALSE(ray.masked);
    }
  }
  // The front ray (azimuth step 360 is 0 rad) of ring 7 is the boresight horizontal (0 rad elevation).
  EXPECT_NEAR(result.frame.rays[flat(layout, 7U, 360U)].azimuth_rad, 0.0F, 1e-6);
  EXPECT_NEAR(result.frame.rays[flat(layout, 7U, 360U)].elevation_rad, 0.0F, 1e-6);
}

TEST(RingCloudToFrameTest, AKnownWallGivesTheExpectedRangePerRing)
{
  // A wall 4.88 m ahead of the sensor (sensor frame, no pitch): range = 4.88 / (cos el cos az).
  const auto layout = benchLayout();
  const auto cloud = makeCloud(layout, [&](uint32_t r, uint32_t s) {
    const auto d = indexDirection(layout, r, s);
    return d[0] > 1e-3 ? at(layout, r, s, 4.88 / d[0]) : noHit(layout, r, s);
  });
  const auto result = ringCloudToFrame(cloud, layout, true);
  for (uint32_t ring = 0U; ring < layout.ring_count; ++ring) {
    const auto & ray = result.frame.rays[flat(layout, ring, 360U)];
    const double elevation = layout.elevation_min_rad + ring * layout.elevation_step_rad;
    EXPECT_EQ(ray.return_kind, RayReturnKind::kReturn) << "ring " << ring;
    EXPECT_NEAR(ray.range_m, 4.88 / std::cos(elevation), 1e-3) << "ring " << ring;
  }
  // Behind the sensor nothing is hit: a qualified no-return (the range is +inf, 50 m >= 40 m).
  const auto & rear = result.frame.rays[flat(layout, 17U, 0U)];
  EXPECT_EQ(rear.return_kind, RayReturnKind::kQualifiedNoReturn);
  EXPECT_TRUE(std::isinf(rear.range_m));
  EXPECT_EQ(result.stats.index_mismatch, 0U);
  EXPECT_GT(result.stats.no_returns, 0U);
  EXPECT_EQ(
    result.stats.returns + result.stats.no_returns + result.stats.invalid, result.stats.points);
}

TEST(RingCloudToFrameTest, ANoHitIsAQualifiedNoReturnFromTheIndexNotFromTheCoordinates)
{
  // All three coordinates are infinite with the signs of the index direction; the angles of the ray
  // are still the index angles.
  const auto layout = benchLayout();
  const auto cloud = makeCloud(layout, [&](uint32_t r, uint32_t s) { return noHit(layout, r, s); });
  const auto result = ringCloudToFrame(cloud, layout, true);
  EXPECT_EQ(result.stats.no_returns, 43200U);
  EXPECT_EQ(result.stats.invalid, 0U);
  const auto & ray = result.frame.rays[flat(layout, 30U, 540U)];
  EXPECT_EQ(ray.return_kind, RayReturnKind::kQualifiedNoReturn);
  EXPECT_NEAR(ray.azimuth_rad, static_cast<float>(kPi / 2.0), 1e-6);
  EXPECT_NEAR(ray.elevation_rad, static_cast<float>(23.0 * kPi / 180.0), 1e-6);
}

TEST(RingCloudToFrameTest, ARangeBelowTheMinimumIsInvalidNeverFree)
{
  // Gazebo writes -inf for a hit nearer than the minimum: the coordinates have the signs opposite to the
  // index direction. It must never read as free space.
  const auto layout = benchLayout();
  const auto cloud = makeCloud(layout, [&](uint32_t r, uint32_t s) {
    const auto d = indexDirection(layout, r, s);
    return std::array<double, 3>{-kInf * nonZero(d[0]), -kInf * nonZero(d[1]), -kInf * nonZero(d[2])};
  });
  const auto result = ringCloudToFrame(cloud, layout, true);
  EXPECT_EQ(result.stats.invalid, 43200U);
  EXPECT_EQ(result.stats.no_returns, 0U);
  EXPECT_EQ(result.frame.rays[flat(layout, 10U, 100U)].return_kind, RayReturnKind::kInvalid);
}

TEST(RingCloudToFrameTest, NaNAndAFiniteRangeOutsideTheLimitsAreInvalidOrNoReturnAsTheLayoutSays)
{
  const auto layout = benchLayout();
  const auto cloud = makeCloud(layout, [&](uint32_t r, uint32_t s) {
    if (r == 0U && s == 0U) { return std::array<double, 3>{kNaN, kNaN, kNaN}; }
    if (r == 0U && s == 1U) { return std::array<double, 3>{kNaN, 1.0, 1.0}; }
    if (r == 0U && s == 2U) { return at(layout, r, s, 0.05); }   // below the minimum range
    if (r == 0U && s == 3U) { return at(layout, r, s, 50.5); }   // beyond the maximum range
    if (r == 0U && s == 4U) { return std::array<double, 3>{0.0, 0.0, 0.0}; }
    return at(layout, r, s, 3.0);
  });
  const auto result = ringCloudToFrame(cloud, layout, true);
  EXPECT_EQ(result.frame.rays[flat(layout, 0U, 0U)].return_kind, RayReturnKind::kInvalid);
  EXPECT_EQ(result.frame.rays[flat(layout, 0U, 1U)].return_kind, RayReturnKind::kInvalid);
  EXPECT_EQ(result.frame.rays[flat(layout, 0U, 2U)].return_kind, RayReturnKind::kInvalid);
  EXPECT_EQ(result.frame.rays[flat(layout, 0U, 3U)].return_kind, RayReturnKind::kQualifiedNoReturn);
  EXPECT_EQ(result.frame.rays[flat(layout, 0U, 4U)].return_kind, RayReturnKind::kInvalid);
  EXPECT_EQ(result.stats.invalid, 4U);
  EXPECT_EQ(result.stats.index_mismatch, 0U);
}

TEST(RingCloudToFrameTest, AScanThatReachesLessThanTheMapRangeNeverCertifiesFreeSpace)
{
  auto layout = benchLayout();
  layout.range_max_m = 30.0F;  // below the 40 m qualified range of the map
  const auto cloud = makeCloud(layout, [&](uint32_t r, uint32_t s) { return noHit(layout, r, s); });
  const auto result = ringCloudToFrame(cloud, layout, true);
  EXPECT_EQ(result.stats.no_returns, 0U);
  EXPECT_EQ(result.stats.invalid, 43200U);
}

TEST(RingCloudToFrameTest, APointInTheWrongDirectionIsInvalidAndCounted)
{
  // A cloud of another layout (here: the ring order reversed) must not be trusted ray by ray.
  const auto layout = benchLayout();
  const auto cloud = makeCloud(layout, [&](uint32_t r, uint32_t s) {
    return at(layout, layout.ring_count - 1U - r, s, 3.0);
  });
  const auto result = ringCloudToFrame(cloud, layout, true);
  // 60 rings is an even count, so no ring maps to itself when the order is reversed.
  EXPECT_EQ(result.stats.index_mismatch, 43200U);
  EXPECT_EQ(result.stats.returns, 0U);
  EXPECT_EQ(result.stats.invalid, 43200U);
  // One point moved by 0.01 rad inside an otherwise correct cloud: only that ray is flagged.
  const auto good = makeCloud(layout, [&](uint32_t r, uint32_t s) {
    auto point = at(layout, r, s, 3.0);
    if (r == 5U && s == 100U) {
      point[1] += 3.0 * 0.01;
    }
    return point;
  });
  const auto one = ringCloudToFrame(good, layout, true);
  EXPECT_EQ(one.stats.index_mismatch, 1U);
  EXPECT_EQ(one.frame.rays[flat(layout, 5U, 100U)].return_kind, RayReturnKind::kInvalid);
  EXPECT_EQ(one.frame.rays[flat(layout, 5U, 101U)].return_kind, RayReturnKind::kReturn);
}

TEST(RingCloudToFrameTest, ACloudThatDoesNotMatchTheLayoutHasNoRaysAndBreaksContinuity)
{
  const auto layout = benchLayout();
  const auto good = makeCloud(layout, [&](uint32_t r, uint32_t s) { return at(layout, r, s, 3.0); });

  auto wrong_width = good;
  wrong_width.width = 360U;
  auto wrong_height = good;
  wrong_height.height = 30U;
  auto big_endian = good;
  big_endian.is_bigendian = true;
  auto no_z = good;
  no_z.fields.erase(no_z.fields.begin() + 2);
  auto double_x = good;
  double_x.fields[0].datatype = PointField::FLOAT64;
  auto short_data = good;
  short_data.data.resize(short_data.data.size() - 1U);
  auto short_step = good;
  short_step.point_step = 8U;
  auto short_row = good;
  short_row.row_step = short_row.point_step * short_row.width - 1U;

  for (const auto * cloud : {&wrong_width, &wrong_height, &big_endian, &no_z, &double_x, &short_data,
                             &short_step, &short_row}) {
    const auto result = ringCloudToFrame(*cloud, layout, true);
    EXPECT_FALSE(result.stats.shape_ok);
    EXPECT_TRUE(result.frame.rays.empty());
    EXPECT_FALSE(result.frame.packet_continuity_valid);
  }
  EXPECT_TRUE(ringCloudToFrame(good, layout, true).stats.shape_ok);
}

TEST(RingCloudToFrameTest, ARowStepWithPaddingIsRead)
{
  const auto layout = benchLayout();
  auto cloud = makeCloud(layout, [&](uint32_t r, uint32_t s) { return at(layout, r, s, 3.0); });
  // Re-pack with 64 bytes of padding at the end of every row.
  const std::size_t old_row = cloud.row_step;
  const std::size_t new_row = old_row + 64U;
  std::vector<uint8_t> padded(new_row * cloud.height, 0xAAU);
  for (uint32_t row = 0U; row < cloud.height; ++row) {
    std::memcpy(padded.data() + row * new_row, cloud.data.data() + row * old_row, old_row);
  }
  cloud.data = padded;
  cloud.row_step = static_cast<uint32_t>(new_row);
  const auto result = ringCloudToFrame(cloud, layout, true);
  ASSERT_TRUE(result.stats.shape_ok);
  EXPECT_EQ(result.stats.returns, 43200U);
}

TEST(RingCloudToFrameTest, AZeroHeaderTimeStaysZeroAndContinuityIsPassedThrough)
{
  const auto layout = benchLayout();
  auto cloud = makeCloud(layout, [&](uint32_t r, uint32_t s) { return at(layout, r, s, 3.0); });
  cloud.header.stamp.sec = 0;
  cloud.header.stamp.nanosec = 0U;
  const auto result = ringCloudToFrame(cloud, layout, false);
  EXPECT_EQ(result.frame.rays[0].timestamp_ns, 0U);
  EXPECT_FALSE(result.frame.packet_continuity_valid);
}

}  // namespace

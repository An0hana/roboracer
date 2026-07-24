#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "opponent_tracker/scan_clusterer.hpp"

namespace opponent_tracker
{
namespace
{

ScanClustererConfig testConfig()
{
  ScanClustererConfig config;
  config.min_range = 0.05;
  config.max_range = 8.0;
  config.breakpoint_base = 0.03;
  config.breakpoint_scale = 2.0;
  config.min_cluster_points = 3U;
  config.min_object_length = 0.02;
  config.max_object_length = 1.0;
  config.max_object_width = 0.8;
  config.min_box_dimension = 0.01;
  config.confidence_full_points = 10U;
  return config;
}

TEST(ScanClusterer, RejectsInvalidConfiguration)
{
  auto config = testConfig();
  config.max_range = config.min_range;
  EXPECT_THROW(ScanClusterer clusterer(config), std::invalid_argument);
}

TEST(ScanClusterer, IgnoresInvalidAndOutOfRangeSamples)
{
  ScanClusterer clusterer(testConfig());
  const std::vector<float> ranges{
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity(),
    0.01F,
    9.0F};

  const auto detections = clusterer.detect(ranges, -0.1, 0.01, 0.0, 30.0);
  EXPECT_TRUE(detections.empty());
}

TEST(ScanClusterer, BuildsOneFiniteOrientedBox)
{
  ScanClusterer clusterer(testConfig());
  std::vector<float> ranges(31U, std::numeric_limits<float>::infinity());
  for (std::size_t index = 12U; index <= 18U; ++index) {
    ranges[index] = 2.0F;
  }

  const auto detections = clusterer.detect(ranges, -0.15, 0.01, 0.0, 30.0);

  ASSERT_EQ(detections.size(), 1U);
  const auto & detection = detections.front();
  EXPECT_TRUE(std::isfinite(detection.x));
  EXPECT_TRUE(std::isfinite(detection.y));
  EXPECT_TRUE(std::isfinite(detection.yaw));
  EXPECT_NEAR(detection.x, 2.0, 0.02);
  EXPECT_NEAR(detection.y, 0.0, 0.02);
  EXPECT_GE(detection.length, detection.width);
  EXPECT_GT(detection.length, 0.05);
  EXPECT_GE(detection.confidence, 0.0);
  EXPECT_LE(detection.confidence, 1.0);
  EXPECT_EQ(detection.point_count, 7U);
  EXPECT_EQ(detection.points.size(), detection.point_count);
}

TEST(ScanClusterer, SeparatesClustersAcrossInvalidBeam)
{
  ScanClusterer clusterer(testConfig());
  const float infinity = std::numeric_limits<float>::infinity();
  const std::vector<float> ranges{
    2.0F, 2.0F, 2.0F, infinity, 3.0F, 3.0F, 3.0F};

  const auto detections = clusterer.detect(ranges, -0.06, 0.02, 0.0, 30.0);
  EXPECT_EQ(detections.size(), 2U);
}

TEST(ScanClusterer, AdaptiveThresholdKeepsDistantNeighborsTogether)
{
  ScanClusterer clusterer(testConfig());
  const std::vector<float> ranges{7.0F, 7.0F, 7.0F};

  const auto detections = clusterer.detect(ranges, -0.01, 0.01, 0.0, 30.0);
  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections.front().point_count, 3U);
}

TEST(ScanClusterer, FiltersClustersWithTooFewPoints)
{
  ScanClusterer clusterer(testConfig());
  const std::vector<float> ranges{2.0F, 2.0F};

  const auto detections = clusterer.detect(ranges, 0.0, 0.02, 0.0, 30.0);
  EXPECT_TRUE(detections.empty());
}

TEST(ScanClusterer, FiltersClearlyLongWall)
{
  ScanClusterer clusterer(testConfig());
  const std::vector<float> ranges(81U, 2.0F);

  const auto detections = clusterer.detect(ranges, -0.4, 0.01, 0.0, 30.0);
  EXPECT_TRUE(detections.empty());
}

}  // namespace
}  // namespace opponent_tracker

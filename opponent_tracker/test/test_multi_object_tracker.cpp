#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "opponent_tracker/multi_object_tracker.hpp"

namespace opponent_tracker
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

Detection detection(double x, double y, double yaw = 0.0)
{
  Detection value;
  value.x = x;
  value.y = y;
  value.yaw = yaw;
  value.length = 0.552;
  value.width = 0.320;
  value.confidence = 1.0;
  value.point_count = 20U;
  return value;
}

MultiObjectTrackerConfig testConfig()
{
  MultiObjectTrackerConfig config;
  config.association_distance = 0.75;
  config.min_confirmed_hits = 2U;
  config.max_missed_time = 0.30;
  config.process_noise = 1.0;
  config.measurement_noise = 0.03;
  config.initial_velocity_stddev = 2.0;
  config.shape_smoothing = 0.5;
  config.dynamic_speed_threshold = 0.10;
  config.max_prediction_dt = 0.10;
  return config;
}

TEST(MultiObjectTracker, RejectsInvalidConfiguration)
{
  auto config = testConfig();
  config.association_distance = 0.0;
  EXPECT_THROW(MultiObjectTracker tracker(config), std::invalid_argument);

  config = testConfig();
  config.heading_exit_speed = config.heading_enter_speed;
  EXPECT_THROW(MultiObjectTracker tracker(config), std::invalid_argument);
}

TEST(MultiObjectTracker, ConfirmsTrackAndKeepsStableId)
{
  MultiObjectTracker tracker(testConfig());

  EXPECT_TRUE(tracker.update({detection(0.0, 0.0)}, 1.0).empty());
  const auto confirmed = tracker.update({detection(0.1, 0.0)}, 1.1);
  ASSERT_EQ(confirmed.size(), 1U);
  const auto id = confirmed.front().id;
  EXPECT_GT(id, 0);
  EXPECT_TRUE(confirmed.front().visible);

  const auto continued = tracker.update({detection(0.2, 0.0)}, 1.2);
  ASSERT_EQ(continued.size(), 1U);
  EXPECT_EQ(continued.front().id, id);
  EXPECT_GT(continued.front().vx, 0.0);
  EXPECT_TRUE(continued.front().dynamic);
}

TEST(MultiObjectTracker, AssociatesTwoSeparatedObjects)
{
  MultiObjectTracker tracker(testConfig());

  tracker.update({detection(0.0, 0.0), detection(3.0, 0.0)}, 1.0);
  const auto initial =
    tracker.update({detection(0.1, 0.0), detection(2.9, 0.0)}, 1.1);
  ASSERT_EQ(initial.size(), 2U);
  EXPECT_NE(initial[0].id, initial[1].id);

  const auto continued =
    tracker.update({detection(2.8, 0.0), detection(0.2, 0.0)}, 1.2);
  ASSERT_EQ(continued.size(), 2U);
  EXPECT_EQ(continued[0].id, initial[0].id);
  EXPECT_EQ(continued[1].id, initial[1].id);
  EXPECT_LT(continued[0].x, continued[1].x);
}

TEST(MultiObjectTracker, HoldsTrackAcrossShortMiss)
{
  MultiObjectTracker tracker(testConfig());
  tracker.update({detection(0.0, 0.0)}, 1.0);
  const auto confirmed = tracker.update({detection(0.1, 0.0)}, 1.1);
  ASSERT_EQ(confirmed.size(), 1U);
  const auto id = confirmed.front().id;

  const auto predicted = tracker.update({}, 1.2);
  ASSERT_EQ(predicted.size(), 1U);
  EXPECT_EQ(predicted.front().id, id);
  EXPECT_FALSE(predicted.front().visible);
  EXPECT_LT(predicted.front().confidence, confirmed.front().confidence);

  const auto reacquired = tracker.update({detection(0.3, 0.0)}, 1.3);
  ASSERT_EQ(reacquired.size(), 1U);
  EXPECT_EQ(reacquired.front().id, id);
  EXPECT_TRUE(reacquired.front().visible);
}

TEST(MultiObjectTracker, ShiftsTrackFrameWithoutChangingIdentity)
{
  MultiObjectTracker tracker(testConfig());
  tracker.update({detection(0.0, 0.0)}, 1.0);
  const auto confirmed = tracker.update({detection(0.1, 0.0)}, 1.1);
  ASSERT_EQ(confirmed.size(), 1U);

  EXPECT_TRUE(tracker.shiftTrackPosition(confirmed.front().id, 0.25, -0.10));
  const auto shifted = tracker.estimates();
  ASSERT_EQ(shifted.size(), 1U);
  EXPECT_EQ(shifted.front().id, confirmed.front().id);
  EXPECT_NEAR(shifted.front().x, confirmed.front().x + 0.25, 1.0e-9);
  EXPECT_NEAR(shifted.front().y, confirmed.front().y - 0.10, 1.0e-9);
  EXPECT_FALSE(tracker.shiftTrackPosition(999, 0.0, 0.0));
}

TEST(MultiObjectTracker, SmoothsObservedDimensions)
{
  auto config = testConfig();
  config.shape_smoothing = 0.25;
  MultiObjectTracker tracker(config);

  auto first = detection(0.0, 0.0);
  first.length = 0.40;
  first.width = 0.20;
  tracker.update({first}, 1.0);

  auto second = detection(0.1, 0.0);
  second.length = 0.80;
  second.width = 0.40;
  const auto estimate = tracker.update({second}, 1.1);
  ASSERT_EQ(estimate.size(), 1U);
  EXPECT_NEAR(estimate.front().length, 0.50, 1.0e-9);
  EXPECT_NEAR(estimate.front().width, 0.25, 1.0e-9);
}

TEST(MultiObjectTracker, DeletesTrackAfterTimeout)
{
  MultiObjectTracker tracker(testConfig());
  tracker.update({detection(0.0, 0.0)}, 1.0);
  ASSERT_EQ(tracker.update({detection(0.1, 0.0)}, 1.1).size(), 1U);
  EXPECT_TRUE(tracker.update({}, 1.5).empty());
}

TEST(MultiObjectTracker, IgnoresInvalidDetection)
{
  MultiObjectTracker tracker(testConfig());
  auto invalid = detection(0.0, 0.0);
  invalid.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(tracker.update({invalid}, 1.0).empty());
}

TEST(MultiObjectTracker, ResetsWhenTimeMovesBackward)
{
  MultiObjectTracker tracker(testConfig());
  tracker.update({detection(0.0, 0.0)}, 2.0);
  const auto confirmed = tracker.update({detection(0.1, 0.0)}, 2.1);
  ASSERT_EQ(confirmed.size(), 1U);

  EXPECT_TRUE(tracker.update({detection(5.0, 0.0)}, 1.0).empty());
  const auto reconfirmed = tracker.update({detection(5.1, 0.0)}, 1.1);
  ASSERT_EQ(reconfirmed.size(), 1U);
  EXPECT_EQ(reconfirmed.front().id, 1);
}

TEST(MultiObjectTracker, KeepsMotionYawWhenContourDirectionChanges)
{
  auto config = testConfig();
  config.heading_enter_speed = 0.05;
  config.heading_exit_speed = 0.02;
  MultiObjectTracker tracker(config);

  tracker.update({detection(0.0, 0.0, 0.5 * kPi)}, 1.0);
  const auto moving =
    tracker.update({detection(0.1, 0.0, 0.5 * kPi)}, 1.1);
  ASSERT_EQ(moving.size(), 1U);
  EXPECT_NEAR(moving.front().yaw, 0.0, 1.0e-6);

  std::vector<TrackEstimate> stopped;
  for (int step = 0; step < 20; ++step) {
    const double contour_yaw = step % 2 == 0 ? 0.0 : 0.5 * kPi;
    stopped = tracker.update(
      {detection(0.1, 0.0, contour_yaw)},
      1.2 + 0.1 * static_cast<double>(step));
  }
  ASSERT_EQ(stopped.size(), 1U);
  EXPECT_NEAR(stopped.front().yaw, 0.0, 1.0e-6);
}

TEST(MultiObjectTracker, LimitsYawChangeDuringTurn)
{
  auto config = testConfig();
  config.heading_enter_speed = 0.01;
  config.heading_exit_speed = 0.0;
  config.heading_smoothing = 1.0;
  config.max_yaw_rate = 0.5;
  MultiObjectTracker tracker(config);

  tracker.update({detection(0.0, 0.0)}, 1.0);
  const auto straight = tracker.update({detection(0.1, 0.0)}, 1.1);
  ASSERT_EQ(straight.size(), 1U);
  EXPECT_NEAR(straight.front().yaw, 0.0, 1.0e-6);

  const auto turning = tracker.update({detection(0.1, 0.1)}, 1.2);
  ASSERT_EQ(turning.size(), 1U);
  EXPECT_LE(std::abs(turning.front().yaw - straight.front().yaw), 0.051);
}

}  // namespace
}  // namespace opponent_tracker

#include <gtest/gtest.h>

#include <cmath>

#include "g1_trajectory/trajectory.hpp"

using g1_trajectory::Trajectory;
using g1_trajectory::TrajectoryOptions;

namespace
{
geometry_msgs::msg::PoseStamped pose(double t, double x, double yaw = 0.0,
                                     const char * frame = "odom")
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = frame;
  p.header.stamp.sec = static_cast<int32_t>(std::floor(t));
  p.header.stamp.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
  p.pose.position.x = x;
  p.pose.orientation.z = std::sin(yaw / 2);
  p.pose.orientation.w = std::cos(yaw / 2);
  return p;
}
}  // namespace

TEST(Trajectory, AddsPoseOnlyAfterMovingOrTurning)
{
  Trajectory trajectory;  // 2 cm, 0.05 rad
  EXPECT_TRUE(trajectory.add(pose(0.0, 0.0)));
  EXPECT_FALSE(trajectory.add(pose(0.1, 0.01)));
  EXPECT_TRUE(trajectory.add(pose(0.2, 0.03)));
  EXPECT_FALSE(trajectory.add(pose(0.3, 0.03, 0.04)));
  EXPECT_TRUE(trajectory.add(pose(0.4, 0.03, 0.06)));
  EXPECT_EQ(trajectory.poses().size(), 3u);
}

TEST(Trajectory, RestartsWhenTimeJumpsBackOrFrameChanges)
{
  Trajectory trajectory;
  trajectory.add(pose(10.0, 0.0));
  trajectory.add(pose(11.0, 1.0));
  EXPECT_TRUE(trajectory.add(pose(10.5, 1.5)));  // small jitter back: kept
  EXPECT_EQ(trajectory.poses().size(), 3u);
  EXPECT_TRUE(trajectory.add(pose(2.0, 0.0)));   // bag looped
  EXPECT_EQ(trajectory.poses().size(), 1u);
  EXPECT_TRUE(trajectory.add(pose(2.1, 0.0, 0.0, "map")));
  EXPECT_EQ(trajectory.poses().size(), 1u);
}

TEST(Trajectory, KeepsAtMostMaxPoses)
{
  TrajectoryOptions options;
  options.max_poses = 3;
  Trajectory trajectory(options);
  for (int i = 0; i < 5; ++i) {
    trajectory.add(pose(i, i));
  }
  ASSERT_EQ(trajectory.poses().size(), 3u);
  EXPECT_DOUBLE_EQ(trajectory.poses().front().pose.position.x, 2.0);
}

TEST(Trajectory, AngleBetweenIgnoresQuaternionSign)
{
  auto a = pose(0, 0, 0.3).pose.orientation;
  auto b = a;
  b.x = -b.x; b.y = -b.y; b.z = -b.z; b.w = -b.w;
  EXPECT_NEAR(g1_trajectory::angle_between(a, b), 0.0, 1e-9);
  EXPECT_NEAR(g1_trajectory::angle_between(a, pose(0, 0, 0.5).pose.orientation), 0.2, 1e-9);
}

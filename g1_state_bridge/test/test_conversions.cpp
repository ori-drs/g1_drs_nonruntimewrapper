#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstring>

#include "g1_state_bridge/conversions.hpp"
#include "g1_state_bridge/g1_model.hpp"
#include "g1_state_bridge/rate_limiter.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_state.hpp"

using namespace g1_state_bridge;

namespace
{
unitree_hg::msg::LowState low_state()
{
  unitree_hg::msg::LowState state;
  for (std::size_t i = 0; i < state.motor_state.size(); ++i) {
    state.motor_state[i].q = 0.1F * i;
    state.motor_state[i].dq = -0.01F * i;
    state.motor_state[i].tau_est = 0.5F * i;
  }
  return state;
}
}  // namespace

TEST(Conversions, StampRoundTrip)
{
  const int64_t ns = 1793801216424837888;
  EXPECT_EQ(to_ns(to_stamp(ns)), ns);
  EXPECT_EQ(to_stamp(1500000000).sec, 1);
  EXPECT_EQ(to_stamp(1500000000).nanosec, 500000000U);
}

TEST(Conversions, ImuQuaternionIsWFirst)
{
  unitree_hg::msg::IMUState imu_state;
  imu_state.quaternion = {0.5F, 0.1F, 0.2F, 0.3F};
  imu_state.gyroscope = {1.0F, 2.0F, 3.0F};
  imu_state.accelerometer = {0.0F, 0.0F, 9.81F};
  const auto imu = imu_state_to_imu(imu_state, to_stamp(7), "imu_in_pelvis");
  EXPECT_FLOAT_EQ(imu.orientation.w, 0.5F);
  EXPECT_FLOAT_EQ(imu.orientation.x, 0.1F);
  EXPECT_FLOAT_EQ(imu.orientation.z, 0.3F);
  EXPECT_FLOAT_EQ(imu.angular_velocity.z, 3.0F);
  EXPECT_FLOAT_EQ(imu.linear_acceleration.z, 9.81F);
  EXPECT_EQ(imu.header.frame_id, "imu_in_pelvis");
  EXPECT_EQ(imu.header.stamp.nanosec, 7U);
}

TEST(Conversions, JointStateMapsMotorIndexAndSkipsEmptyNames)
{
  auto names = kG1JointNames;
  names[13] = "";
  const auto joint_state = motor_states_to_joint_state(low_state().motor_state, names, to_stamp(0));
  ASSERT_EQ(joint_state.name.size(), kG1JointNames.size() - 1);
  const auto it = std::find(joint_state.name.begin(), joint_state.name.end(), "right_knee_joint");
  const auto index = static_cast<std::size_t>(it - joint_state.name.begin());
  EXPECT_FLOAT_EQ(joint_state.position[index], 0.9F);
  EXPECT_FLOAT_EQ(joint_state.velocity[index], -0.09F);
  EXPECT_FLOAT_EQ(joint_state.effort[index], 4.5F);
  EXPECT_EQ(
    std::find(joint_state.name.begin(), joint_state.name.end(), "waist_roll_joint"),
    joint_state.name.end());
}

TEST(Conversions, WirelessRemoteToJoy)
{
  std::array<uint8_t, 40> raw{};
  raw[0] = 'U';
  raw[1] = 'Q';
  const uint16_t buttons = (1U << 8) | 1U;   // A, R1
  std::memcpy(raw.data() + kRemoteButtonsOffset, &buttons, sizeof(buttons));
  const float axes[] = {0.1F, 0.2F, 0.3F, 0.4F, 0.5F};   // raw order: lx rx ry l2 ly
  std::memcpy(raw.data() + kRemoteAxesOffset, axes, sizeof(axes));
  const auto joy = wireless_remote_to_joy(raw, to_stamp(0));
  const std::vector<float> expected = {0.1F, 0.5F, 0.2F, 0.3F, 0.4F};   // lx ly rx ry l2
  EXPECT_EQ(joy.axes, expected);
  ASSERT_EQ(joy.buttons.size(), 16U);
  EXPECT_EQ(joy.buttons[8], 1);   // A
  EXPECT_EQ(joy.buttons[0], 1);   // R1
  int pressed = 0;
  for (const auto button : joy.buttons) {
    pressed += button;
  }
  EXPECT_EQ(pressed, 2);
}

TEST(Conversions, DiagnosticsFlagMotorErrors)
{
  auto state = low_state();
  state.motor_state[3].motorstate = 4;
  const auto diagnostics = motor_states_to_diagnostics(state.motor_state, kG1JointNames, to_stamp(0));
  ASSERT_EQ(diagnostics.status.size(), kG1JointNames.size());
  EXPECT_EQ(diagnostics.status[3].level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(diagnostics.status[3].message, "motorstate=0x00000004");
  EXPECT_EQ(diagnostics.status[0].level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(diagnostics.status[0].name, "g1/motor/left_hip_pitch_joint");
}

TEST(Conversions, OdometryRotatesWorldVelocityIntoBody)
{
  unitree_go::msg::SportModeState state;
  state.position = {1.0F, 2.0F, 0.75F};
  const float half = std::sqrt(0.5F);   // yaw +90 deg
  state.imu_state.quaternion = {half, 0.0F, 0.0F, half};
  state.imu_state.gyroscope = {0.0F, 0.0F, 0.3F};
  state.velocity = {0.0F, 1.0F, 0.0F};   // world +y = body +x after a +90 deg yaw

  const auto odom = sport_state_to_odometry(state, to_stamp(5), "odom", "pelvis", true);
  EXPECT_EQ(odom.header.frame_id, "odom");
  EXPECT_EQ(odom.child_frame_id, "pelvis");
  EXPECT_FLOAT_EQ(odom.pose.pose.position.y, 2.0F);
  EXPECT_NEAR(odom.pose.pose.orientation.z, half, 1e-6);
  EXPECT_NEAR(odom.twist.twist.linear.x, 1.0, 1e-6);
  EXPECT_NEAR(odom.twist.twist.linear.y, 0.0, 1e-6);
  EXPECT_FLOAT_EQ(odom.twist.twist.angular.z, 0.3F);

  const auto body = sport_state_to_odometry(state, to_stamp(5), "odom", "pelvis", false);
  EXPECT_NEAR(body.twist.twist.linear.y, 1.0, 1e-6);

  const auto transform = odometry_to_transform(odom);
  EXPECT_EQ(transform.header.frame_id, "odom");
  EXPECT_EQ(transform.child_frame_id, "pelvis");
  EXPECT_FLOAT_EQ(transform.transform.translation.x, 1.0F);
  EXPECT_NEAR(transform.transform.rotation.w, half, 1e-6);
}

TEST(RateLimiter, KeepsAverageRate)
{
  RateLimiter limiter(10.0);
  int kept = 0;
  for (int i = 0; i < 2000; ++i) {   // 1 kHz input for 2 s
    kept += limiter.allow(static_cast<int64_t>(i) * 1000000) ? 1 : 0;
  }
  EXPECT_EQ(kept, 20);
  RateLimiter keep_all(0.0);
  for (int i = 0; i < 5; ++i) {
    EXPECT_TRUE(keep_all.allow(i));
  }
}

TEST(G1Model, UrdfTable)
{
  EXPECT_EQ(urdf_model_for_mode_machine(6), "g1_29dof_lock_waist_rev_1_0");
  EXPECT_EQ(urdf_model_for_mode_machine(99), "");
  EXPECT_EQ(kG1JointNames.size(), 29U);
}

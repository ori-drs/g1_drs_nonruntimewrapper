#include "g1_state_bridge/conversions.hpp"

#include <cstring>
#include <map>

#include "g1_state_bridge/g1_model.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace g1_state_bridge
{

namespace
{
constexpr int64_t kNsPerSec = 1000000000;
}  // namespace

std::string urdf_model_for_mode_machine(int mode_machine)
{
  static const std::map<int, std::string> models = {
    {4, "g1_23dof_rev_1_0"}, {5, "g1_29dof_rev_1_0"}, {6, "g1_29dof_lock_waist_rev_1_0"},
    {9, "g1_dual_arm"}, {10, "g1_23dof_mode_10"}, {11, "g1_29dof_mode_11"},
    {12, "g1_29dof_mode_12"}, {13, "g1_29dof_mode_13"}, {14, "g1_29dof_mode_14"},
    {15, "g1_29dof_mode_15"}, {16, "g1_29dof_mode_16"}, {18, "g1_29dof_mode_18"},
    {23, "g1_pl"},
  };
  const auto it = models.find(mode_machine);
  return it == models.end() ? "" : it->second;
}

builtin_interfaces::msg::Time to_stamp(int64_t stamp_ns)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(stamp_ns / kNsPerSec);
  stamp.nanosec = static_cast<uint32_t>(stamp_ns % kNsPerSec);
  return stamp;
}

int64_t to_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * kNsPerSec + stamp.nanosec;
}

sensor_msgs::msg::Joy wireless_remote_to_joy(
  const std::array<uint8_t, 40> & wireless_remote, const builtin_interfaces::msg::Time & stamp)
{
  // Raw layout is little endian, like the robot and the PCs it runs on.
  uint16_t button_word = 0;
  std::memcpy(&button_word, wireless_remote.data() + kRemoteButtonsOffset, sizeof(button_word));
  std::array<float, kRemoteAxisCount> raw_axes{};
  std::memcpy(raw_axes.data(), wireless_remote.data() + kRemoteAxesOffset, sizeof(raw_axes));

  sensor_msgs::msg::Joy joy;
  joy.header.stamp = stamp;
  for (const std::size_t raw_index : kJoyAxisFromRaw) {
    joy.axes.push_back(raw_axes[raw_index]);
  }
  for (std::size_t bit = 0; bit < kRemoteButtonNames.size(); ++bit) {
    joy.buttons.push_back((button_word >> bit) & 1);
  }
  return joy;
}

nav_msgs::msg::Odometry sport_state_to_odometry(
  const unitree_go::msg::SportModeState & state, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id, const std::string & child_frame_id, bool velocity_in_world)
{
  const auto & q = state.imu_state.quaternion;   // (w, x, y, z)
  const tf2::Quaternion orientation(q[1], q[2], q[3], q[0]);
  tf2::Vector3 velocity(state.velocity[0], state.velocity[1], state.velocity[2]);
  if (velocity_in_world) {
    velocity = tf2::quatRotate(orientation.inverse(), velocity);
  }

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = frame_id;
  odom.child_frame_id = child_frame_id;
  odom.pose.pose.position.x = state.position[0];
  odom.pose.pose.position.y = state.position[1];
  odom.pose.pose.position.z = state.position[2];
  odom.pose.pose.orientation.w = q[0];
  odom.pose.pose.orientation.x = q[1];
  odom.pose.pose.orientation.y = q[2];
  odom.pose.pose.orientation.z = q[3];
  odom.twist.twist.linear.x = velocity.x();
  odom.twist.twist.linear.y = velocity.y();
  odom.twist.twist.linear.z = velocity.z();
  odom.twist.twist.angular.x = state.imu_state.gyroscope[0];
  odom.twist.twist.angular.y = state.imu_state.gyroscope[1];
  odom.twist.twist.angular.z = state.imu_state.gyroscope[2];
  return odom;
}

geometry_msgs::msg::TransformStamped odometry_to_transform(const nav_msgs::msg::Odometry & odom)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header = odom.header;
  transform.child_frame_id = odom.child_frame_id;
  transform.transform.translation.x = odom.pose.pose.position.x;
  transform.transform.translation.y = odom.pose.pose.position.y;
  transform.transform.translation.z = odom.pose.pose.position.z;
  transform.transform.rotation = odom.pose.pose.orientation;
  return transform;
}

}  // namespace g1_state_bridge

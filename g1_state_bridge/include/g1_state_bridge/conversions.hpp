// Pure conversions from Unitree G1 messages to standard ROS 2 messages (no I/O, no state).
// Same semantics as g1_drs/unitree_conversions.py, so live and converted bags match.
#pragma once

#include <array>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

namespace g1_state_bridge
{

builtin_interfaces::msg::Time to_stamp(int64_t stamp_ns);
int64_t to_ns(const builtin_interfaces::msg::Time & stamp);

// unitree_hg/IMUState or unitree_go/IMUState -> sensor_msgs/Imu.
// Unitree quaternions are (w, x, y, z). Covariances stay 0 ("unknown"): Unitree reports none.
template<class ImuStateT>
sensor_msgs::msg::Imu imu_state_to_imu(
  const ImuStateT & imu_state, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  sensor_msgs::msg::Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = frame_id;
  imu.orientation.w = imu_state.quaternion[0];
  imu.orientation.x = imu_state.quaternion[1];
  imu.orientation.y = imu_state.quaternion[2];
  imu.orientation.z = imu_state.quaternion[3];
  imu.angular_velocity.x = imu_state.gyroscope[0];
  imu.angular_velocity.y = imu_state.gyroscope[1];
  imu.angular_velocity.z = imu_state.gyroscope[2];
  imu.linear_acceleration.x = imu_state.accelerometer[0];
  imu.linear_acceleration.y = imu_state.accelerometer[1];
  imu.linear_acceleration.z = imu_state.accelerometer[2];
  return imu;
}

// LowState.motor_state -> sensor_msgs/JointState (q, dq, tau_est).
// joint_names[i] names motor i; an empty name skips that motor.
template<class MotorStatesT>
sensor_msgs::msg::JointState motor_states_to_joint_state(
  const MotorStatesT & motor_states, const std::vector<std::string> & joint_names,
  const builtin_interfaces::msg::Time & stamp)
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = stamp;
  for (std::size_t i = 0; i < joint_names.size() && i < motor_states.size(); ++i) {
    if (joint_names[i].empty()) {
      continue;
    }
    joint_state.name.push_back(joint_names[i]);
    joint_state.position.push_back(motor_states[i].q);
    joint_state.velocity.push_back(motor_states[i].dq);
    joint_state.effort.push_back(motor_states[i].tau_est);
  }
  return joint_state;
}

// LowState.motor_state -> one DiagnosticStatus per named motor. Values are raw; a non-zero
// `motorstate` (Unitree error flags) is WARN, temperatures are not judged.
template<class MotorStatesT>
diagnostic_msgs::msg::DiagnosticArray motor_states_to_diagnostics(
  const MotorStatesT & motor_states, const std::vector<std::string> & joint_names,
  const builtin_interfaces::msg::Time & stamp)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  using diagnostic_msgs::msg::KeyValue;
  auto key_value = [](const std::string & key, const std::string & value) {
      KeyValue kv;
      kv.key = key;
      kv.value = value;
      return kv;
    };

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = stamp;
  for (std::size_t i = 0; i < joint_names.size() && i < motor_states.size(); ++i) {
    if (joint_names[i].empty()) {
      continue;
    }
    const auto & motor = motor_states[i];
    const uint32_t error_flags = motor.motorstate;
    char flags_text[24];
    std::snprintf(flags_text, sizeof(flags_text), "motorstate=0x%08x", error_flags);
    char voltage_text[24];
    std::snprintf(voltage_text, sizeof(voltage_text), "%.3f", motor.vol);

    DiagnosticStatus status;
    status.level = error_flags == 0 ? DiagnosticStatus::OK : DiagnosticStatus::WARN;
    status.name = "g1/motor/" + joint_names[i];
    status.message = error_flags == 0 ? "" : flags_text;
    status.hardware_id = "motor_" + std::to_string(i);
    status.values = {
      key_value("mode", std::to_string(motor.mode)),
      key_value("temperature_0", std::to_string(motor.temperature[0])),
      key_value("temperature_1", std::to_string(motor.temperature[1])),
      key_value("voltage", voltage_text),
      key_value("motorstate", std::to_string(error_flags)),
    };
    diagnostics.status.push_back(status);
  }
  return diagnostics;
}

// LowState.wireless_remote (40 raw bytes) -> sensor_msgs/Joy:
// axes (lx, ly, rx, ry, l2), buttons in kRemoteButtonNames order.
sensor_msgs::msg::Joy wireless_remote_to_joy(
  const std::array<uint8_t, 40> & wireless_remote, const builtin_interfaces::msg::Time & stamp);

// unitree_go/SportModeState (Unitree's odometer) -> nav_msgs/Odometry.
// Pose: `position` and imu_state.quaternion. Twist (child frame): `velocity`, rotated into the
// body frame when velocity_in_world, and imu_state.gyroscope. Covariances stay 0 ("unknown").
nav_msgs::msg::Odometry sport_state_to_odometry(
  const unitree_go::msg::SportModeState & state, const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id, const std::string & child_frame_id, bool velocity_in_world);

geometry_msgs::msg::TransformStamped odometry_to_transform(const nav_msgs::msg::Odometry & odom);

}  // namespace g1_state_bridge

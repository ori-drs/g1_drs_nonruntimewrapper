// Static facts about the Unitree G1 used by the state bridge (same tables as g1_drs/g1_model.py).
#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace g1_state_bridge
{

// Motor index (position in LowState.motor_state) -> URDF joint name, G1 29-DoF SDK layout.
// On waist-locked bodies the waist joints are fixed in the URDF; robot_state_publisher
// ignores names it does not know.
inline const std::vector<std::string> kG1JointNames = {
  "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",          // 0-2
  "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",         // 3-5
  "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",       // 6-8
  "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",      // 9-11
  "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",                   // 12-14
  "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",     // 15-17
  "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",  // 18-21
  "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",  // 22-24
  "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",  // 25-28
};

// Unitree wireless remote (LowState.wireless_remote, 40 bytes, little endian):
//   uint8 head[2] | uint16 buttons | float lx, rx, ry, l2, ly | uint8 idle[16]
constexpr std::size_t kRemoteButtonsOffset = 2;
constexpr std::size_t kRemoteAxesOffset = 4;
constexpr std::size_t kRemoteAxisCount = 5;
// Bit i of the button word, LSB first.
inline const std::array<const char *, 16> kRemoteButtonNames = {
  "R1", "L1", "start", "select", "R2", "L2", "F1", "F2",
  "A", "B", "X", "Y", "up", "right", "down", "left"};
// sensor_msgs/Joy axes are (lx, ly, rx, ry, l2); index of each in the raw (lx, rx, ry, l2, ly).
constexpr std::array<std::size_t, kRemoteAxisCount> kJoyAxisFromRaw = {0, 4, 1, 2, 3};

// LowState.mode_machine -> URDF model (unitree_ros/robots/g1_description/README.md).
std::string urdf_model_for_mode_machine(int mode_machine);

}  // namespace g1_state_bridge

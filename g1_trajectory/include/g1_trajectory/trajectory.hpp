// Trajectory of a moving frame, built from odometry poses.
#pragma once

#include <cmath>
#include <cstddef>
#include <deque>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace g1_trajectory
{

struct TrajectoryOptions
{
  double min_distance = 0.02;     // [m] add a pose after moving this far ...
  double min_angle = 0.05;        // [rad] ... or turning this much
  std::size_t max_poses = 20000;  // oldest poses are dropped beyond this (0 = no limit)
  double max_time_jump = 1.0;     // [s] restart when the stamps go back by more (bag loop)
};

inline double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
}

// Rotation angle between two orientations [rad]: angle of the relative rotation conj(a) * b,
// with atan2 for full precision near zero.
inline double angle_between(const geometry_msgs::msg::Quaternion & a,
                            const geometry_msgs::msg::Quaternion & b)
{
  const double w = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
  const double x = a.w * b.x - a.x * b.w - a.y * b.z + a.z * b.y;
  const double y = a.w * b.y + a.x * b.z - a.y * b.w - a.z * b.x;
  const double z = a.w * b.z - a.x * b.y + a.y * b.x - a.z * b.w;
  return 2.0 * std::atan2(std::sqrt(x * x + y * y + z * z), std::abs(w));
}

class Trajectory
{
public:
  explicit Trajectory(const TrajectoryOptions & options = TrajectoryOptions())
  : options_(options) {}

  // Returns true if the pose was added (the trajectory changed).
  bool add(const geometry_msgs::msg::PoseStamped & pose)
  {
    if (!poses_.empty()) {
      const auto & last = poses_.back();
      const bool new_frame = pose.header.frame_id != last.header.frame_id;
      const bool time_jump =
        stamp_seconds(pose.header.stamp) < stamp_seconds(last.header.stamp) - options_.max_time_jump;
      if (new_frame || time_jump) {
        poses_.clear();
      } else {
        const double dx = pose.pose.position.x - last.pose.position.x;
        const double dy = pose.pose.position.y - last.pose.position.y;
        const double dz = pose.pose.position.z - last.pose.position.z;
        const bool moved = std::sqrt(dx * dx + dy * dy + dz * dz) >= options_.min_distance;
        const bool turned =
          angle_between(pose.pose.orientation, last.pose.orientation) >= options_.min_angle;
        if (!moved && !turned) {
          return false;
        }
      }
    }
    poses_.push_back(pose);
    if (options_.max_poses > 0 && poses_.size() > options_.max_poses) {
      poses_.pop_front();
    }
    return true;
  }

  void clear() {poses_.clear();}
  const std::deque<geometry_msgs::msg::PoseStamped> & poses() const {return poses_;}

private:
  TrajectoryOptions options_;
  std::deque<geometry_msgs::msg::PoseStamped> poses_;
};

}  // namespace g1_trajectory

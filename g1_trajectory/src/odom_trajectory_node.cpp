// odom (nav_msgs/Odometry) -> trajectory (nav_msgs/Path), for RViz.
//
// A pose is added each time the robot has moved min_distance or turned min_angle; the path is
// published at publish_rate_hz when it changed. The publisher is transient-local, so RViz gets
// the whole path when it starts late. The path restarts when the odometry frame changes or the
// stamps jump back (a looping bag); `ros2 service call /odom_trajectory/reset std_srvs/srv/Empty`
// clears it by hand.

#include <chrono>
#include <memory>

#include "g1_trajectory/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

namespace g1_trajectory
{

class OdomTrajectory : public rclcpp::Node
{
public:
  OdomTrajectory()
  : Node("odom_trajectory")
  {
    TrajectoryOptions options;
    options.min_distance = declare_parameter<double>("min_distance", options.min_distance);
    options.min_angle = declare_parameter<double>("min_angle", options.min_angle);
    options.max_poses = static_cast<std::size_t>(
      declare_parameter<int>("max_poses", static_cast<int>(options.max_poses)));
    trajectory_ = Trajectory(options);
    const double rate_hz = declare_parameter<double>("publish_rate_hz", 5.0);

    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "trajectory", rclcpp::QoS(1).reliable().transient_local());
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose = msg->pose.pose;
        changed_ = trajectory_.add(pose) || changed_;
      });
    reset_service_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
             std::shared_ptr<std_srvs::srv::Empty::Response>) {
        trajectory_.clear();
        changed_ = true;
        RCLCPP_INFO(get_logger(), "Trajectory cleared");
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(rate_hz, 0.1)), [this]() {publish();});
  }

private:
  void publish()
  {
    if (!changed_) {
      return;
    }
    changed_ = false;
    nav_msgs::msg::Path path;
    const auto & poses = trajectory_.poses();
    if (!poses.empty()) {
      path.header = poses.back().header;
    }
    path.poses.assign(poses.begin(), poses.end());
    path_publisher_->publish(path);
  }

  Trajectory trajectory_;
  bool changed_ = false;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace g1_trajectory

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g1_trajectory::OdomTrajectory>());
  rclcpp::shutdown();
  return 0;
}

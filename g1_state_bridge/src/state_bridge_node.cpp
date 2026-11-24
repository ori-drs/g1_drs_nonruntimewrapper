// g1_state_bridge: Unitree G1 state topics (no header / empty stamp) -> standard ROS 2 messages.
//
// Every output is stamped with the DDS source timestamp of the input sample, i.e. PC1's clock
// when PC1 published it (falls back to this node's clock if the middleware gives none).
//   /lowstate      (unitree_hg/LowState)       -> joint states, pelvis IMU, remote, diagnostics
//   /secondary_imu (unitree_hg/IMUState)       -> torso IMU
//   /odommodestate (unitree_go/SportModeState) -> odometry + TF odom -> pelvis

#include <memory>
#include <string>
#include <vector>

#include "g1_state_bridge/conversions.hpp"
#include "g1_state_bridge/g1_model.hpp"
#include "g1_state_bridge/rate_limiter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace g1_state_bridge
{

using diagnostic_msgs::msg::DiagnosticArray;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::JointState;
using sensor_msgs::msg::Joy;
using unitree_go::msg::SportModeState;
using unitree_hg::msg::IMUState;
using unitree_hg::msg::LowState;

// One published output: disabled when its topic is empty, rate-limited by message stamp.
template<class MsgT>
struct Output
{
  typename rclcpp::Publisher<MsgT>::SharedPtr publisher;
  RateLimiter limiter;

  bool due(int64_t stamp_ns) {return publisher && limiter.allow(stamp_ns);}
};

class StateBridge : public rclcpp::Node
{
public:
  explicit StateBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("g1_state_bridge", options)
  {
    use_source_stamp_ = declare_parameter<std::string>("stamp_source", "source") == "source";
    expected_robot_model_ = declare_parameter<std::string>("expected_robot_model", "");

    joint_names_ = declare_parameter<std::vector<std::string>>("lowstate.joint_names", kG1JointNames);
    joint_states_ = make_output<JointState>("lowstate.joint_states", "/joint_states", 0.0);
    pelvis_imu_ = make_output<Imu>("lowstate.imu", "/imu/pelvis", 0.0);
    pelvis_imu_frame_ = declare_parameter<std::string>("lowstate.imu_frame_id", "imu_in_pelvis");
    remote_ = make_output<Joy>("lowstate.remote", "/remote_controller", 50.0);
    diagnostics_ = make_output<DiagnosticArray>("lowstate.diagnostics", "/diagnostics", 10.0);

    torso_imu_ = make_output<Imu>("secondary_imu.output", "/imu/torso", 0.0);
    torso_imu_frame_ = declare_parameter<std::string>("secondary_imu.frame_id", "imu_in_torso");

    odom_ = make_output<Odometry>("odom.output", "/odom", 0.0);
    odom_frame_ = declare_parameter<std::string>("odom.frame_id", "odom");
    base_frame_ = declare_parameter<std::string>("odom.child_frame_id", "pelvis");
    velocity_in_world_ =
      declare_parameter<std::string>("odom.velocity_frame", "world") == "world";
    tf_limiter_ = RateLimiter(declare_parameter<double>("odom.tf_rate_hz", 100.0));
    if (declare_parameter<bool>("odom.publish_tf", true)) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    // Unitree publishes best effort; SensorDataQoS also matches reliable publishers.
    const auto qos = rclcpp::SensorDataQoS();
    const auto low_state_topic = declare_parameter<std::string>("lowstate.topic", "/lowstate");
    const auto secondary_imu_topic =
      declare_parameter<std::string>("secondary_imu.topic", "/secondary_imu");
    const auto odom_state_topic = declare_parameter<std::string>("odom.topic", "/odommodestate");
    if (!low_state_topic.empty()) {
      low_state_sub_ = create_subscription<LowState>(
        low_state_topic, qos,
        [this](const std::shared_ptr<const LowState> msg, const rclcpp::MessageInfo & info) {
          on_low_state(*msg, info);
        });
    }
    if (!secondary_imu_topic.empty()) {
      secondary_imu_sub_ = create_subscription<IMUState>(
        secondary_imu_topic, qos,
        [this](const std::shared_ptr<const IMUState> msg, const rclcpp::MessageInfo & info) {
          on_secondary_imu(*msg, info);
        });
    }
    if (!odom_state_topic.empty()) {
      odom_state_sub_ = create_subscription<SportModeState>(
        odom_state_topic, qos,
        [this](const std::shared_ptr<const SportModeState> msg, const rclcpp::MessageInfo & info) {
          on_odom_state(*msg, info);
        });
    }
    RCLCPP_INFO(
      get_logger(), "Bridging %s, %s, %s; stamps from %s", low_state_topic.c_str(),
      secondary_imu_topic.c_str(), odom_state_topic.c_str(),
      use_source_stamp_ ? "DDS source timestamps (PC1 clock)" : "receive time");
  }

private:
  // Declares "<prefix>_topic" and "<prefix>_rate_hz"; an empty topic disables the output.
  template<class MsgT>
  Output<MsgT> make_output(const std::string & prefix, const std::string & topic, double rate_hz)
  {
    Output<MsgT> output;
    const auto name = declare_parameter<std::string>(prefix + "_topic", topic);
    output.limiter = RateLimiter(declare_parameter<double>(prefix + "_rate_hz", rate_hz));
    if (!name.empty()) {
      output.publisher = create_publisher<MsgT>(name, rclcpp::QoS(10));
    }
    return output;
  }

  int64_t stamp_ns(const rclcpp::MessageInfo & info)
  {
    const int64_t source_ns = info.get_rmw_message_info().source_timestamp;
    if (use_source_stamp_ && source_ns > 0) {
      return source_ns;
    }
    if (use_source_stamp_) {
      RCLCPP_WARN_ONCE(get_logger(), "No DDS source timestamp; stamping with receive time");
    }
    return now().nanoseconds();
  }

  void on_low_state(const LowState & msg, const rclcpp::MessageInfo & info)
  {
    check_robot_model(msg.mode_machine);
    const int64_t time_ns = stamp_ns(info);
    const auto stamp = to_stamp(time_ns);
    if (joint_states_.due(time_ns)) {
      joint_states_.publisher->publish(
        motor_states_to_joint_state(msg.motor_state, joint_names_, stamp));
    }
    if (pelvis_imu_.due(time_ns)) {
      pelvis_imu_.publisher->publish(imu_state_to_imu(msg.imu_state, stamp, pelvis_imu_frame_));
    }
    if (remote_.due(time_ns)) {
      remote_.publisher->publish(wireless_remote_to_joy(msg.wireless_remote, stamp));
    }
    if (diagnostics_.due(time_ns)) {
      diagnostics_.publisher->publish(
        motor_states_to_diagnostics(msg.motor_state, joint_names_, stamp));
    }
  }

  void on_secondary_imu(const IMUState & msg, const rclcpp::MessageInfo & info)
  {
    const int64_t time_ns = stamp_ns(info);
    if (torso_imu_.due(time_ns)) {
      torso_imu_.publisher->publish(imu_state_to_imu(msg, to_stamp(time_ns), torso_imu_frame_));
    }
  }

  void on_odom_state(const SportModeState & msg, const rclcpp::MessageInfo & info)
  {
    const int64_t time_ns = stamp_ns(info);
    const bool publish_odom = odom_.due(time_ns);
    const bool publish_tf = tf_broadcaster_ && tf_limiter_.allow(time_ns);
    if (!publish_odom && !publish_tf) {
      return;
    }
    const auto odom = sport_state_to_odometry(
      msg, to_stamp(time_ns), odom_frame_, base_frame_, velocity_in_world_);
    if (publish_odom) {
      odom_.publisher->publish(odom);
    }
    if (publish_tf) {
      tf_broadcaster_->sendTransform(odometry_to_transform(odom));
    }
  }

  // Log the robot configuration once (and on change); warn if the launched URDF differs.
  void check_robot_model(int mode_machine)
  {
    if (mode_machine == mode_machine_) {
      return;
    }
    mode_machine_ = mode_machine;
    const auto model = urdf_model_for_mode_machine(mode_machine);
    RCLCPP_INFO(
      get_logger(), "Robot mode_machine=%d -> URDF '%s'", mode_machine,
      model.empty() ? "unknown" : model.c_str());
    if (!expected_robot_model_.empty() && model != expected_robot_model_) {
      RCLCPP_WARN(
        get_logger(), "Launched URDF '%s' does not match this robot ('%s')",
        expected_robot_model_.c_str(), model.c_str());
    }
  }

  bool use_source_stamp_;
  std::string expected_robot_model_;
  std::vector<std::string> joint_names_;
  std::string pelvis_imu_frame_;
  std::string torso_imu_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool velocity_in_world_;
  int mode_machine_ = -1;

  Output<JointState> joint_states_;
  Output<Imu> pelvis_imu_;
  Output<Joy> remote_;
  Output<DiagnosticArray> diagnostics_;
  Output<Imu> torso_imu_;
  Output<Odometry> odom_;
  RateLimiter tf_limiter_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<LowState>::SharedPtr low_state_sub_;
  rclcpp::Subscription<IMUState>::SharedPtr secondary_imu_sub_;
  rclcpp::Subscription<SportModeState>::SharedPtr odom_state_sub_;
};

}  // namespace g1_state_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g1_state_bridge::StateBridge>());
  rclcpp::shutdown();
  return 0;
}

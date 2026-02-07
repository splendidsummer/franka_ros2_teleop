#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>

namespace franka_joystick {

/**
 * @class JoystickPosePublisher
 * @brief 读取 /joy 话题，将摇杆输入映射到笛卡尔空间的位姿，并发布到 /cartesian_pose_command。
 */
class JoystickPosePublisher : public rclcpp::Node {
public:
  JoystickPosePublisher();

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr   joy_subscriber_;
  geometry_msgs::msg::PoseStamped                           current_pose_;
  rclcpp::Time                                              last_joy_time_{0, 0, RCL_ROS_TIME};

  static constexpr double TRANSLATION_SPEED = 0.1; // m/s per joystick unit
  static constexpr double ROTATION_SPEED    = 0.5; // rad/s per joystick unit
};

}  // namespace franka_joystick
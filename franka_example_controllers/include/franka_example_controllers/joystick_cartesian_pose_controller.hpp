#pragma once

#include <Eigen/Dense>
#include <string>
#include <memory>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <franka_example_controllers/robot_utils.hpp>
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <franka_msgs/srv/set_full_collision_behavior.hpp>


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {

class JoystickCartesianPoseExampleController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  // Franka semantic interface
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  // Pose targets
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_;

  // State
  const bool k_elbow_activated_{false};
  bool initialization_flag_{true};
  double elapsed_time_{0.0};
  double initial_robot_time_{0.0};
  double robot_time_{0.0};
  std::string robot_description_;
  std::string arm_id_;

  // Loaned interfaces that need to be stored
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces_;

  // Joystick control
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  geometry_msgs::msg::PoseStamped current_pose_;
  rclcpp::Time last_joy_time_;
  
  // Joystick control parameters
  const double TRANSLATION_SPEED = 0.1; // meters per second
  const double ROTATION_SPEED = 0.5;    // radians per second
  const double DEADZONE = 0.10;         // joystick deadzone

  // Callback for joystick input
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
};

}  // namespace franka_example_controllers

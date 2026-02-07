#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include "franka_semantic_components/franka_robot_state.hpp"
 
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace franka_example_controllers {
/**
 * The Franka RL controller is a controller that publishes joint states
 * and can be used for reinforcement learning tasks.
 */
class FrankaRLController : public controller_interface::ControllerInterface {
 public:
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
        const override;
    [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
        const override;

    controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:
    rclcpp::Subscription<franka_semantic_components::FrankaRobotState>::SharedPtr robot_state_sub_;        

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_position_command_pub_;
    // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_command_pub_;
    // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_effort_command_pub_;
    rclcpp::Subscriber<std_msgs::msg::Float64MultiArray>::SharedPtr _position_command_sub_
    rclcpp::Subscriber<std_msgs::msg::Float64MultiArray>::SharedPtr _init__joint_state_sub_;

    std::array<double, 7> arm_action_{0, 0, 0, 0, 0, 0, 0};  // pub action filtered by controller 
    std::array<double, 7> rl_arm_action_{0, 0, 0, 0, 0, 0, 0}; // sub action from RL node published 
    // msg.data.assign(rl_arm_action_.begin(), rl_arm_action_.end());
    
    std::string arm_id_;  // \todo keep it? 
    bool is_gazebo_{false};
    std::string robot_description_;
    const int num_joints = 7;
    std::array<double, 7> initial_q_{0, 0, 0, 0, 0, 0, 0};
    double elapsed_time_ = 0.0;
    double initial_robot_time_ = 0.0;
    double robot_time_ = 0.0;
    double trajectory_period_ = 0.001;
    bool initialization_flag_{true};
    rclcpp::Time start_time_;
    Eigen::Vector3d end_effector_position_{0.0, 0.0, 0.0};
    Eigen::Quaterniond end_effector_orientation_{1.0, 0.0, 0.0, 0.0};
};

}  // namespace franka_example_controllers

constexpr double EE_X_MIN = 0.2;
constexpr double EE_X_MAX = 0.8;
constexpr double EE_Y_MIN = -0.4;
constexpr double EE_Y_MAX = 0.4;
constexpr double EE_Z_MIN = 0.0;
constexpr double EE_Z_MAX = 0.6;

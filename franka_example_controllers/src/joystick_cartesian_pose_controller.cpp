#include "franka_example_controllers/joystick_cartesian_pose_controller.hpp"
#include <franka_example_controllers/default_robot_behavior_utils.hpp>

#include <pluginlib/class_list_macros.hpp>
#include <cassert>
#include <cmath>
#include <exception>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration JoystickCartesianPoseExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (franka_cartesian_pose_) {
    config.names = franka_cartesian_pose_->get_command_interface_names();
    for (const auto& name : config.names) {
      std::cout << "Command interface config name: " << name << std::endl;
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration JoystickCartesianPoseExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  if (franka_cartesian_pose_) {
    config.names = franka_cartesian_pose_->get_state_interface_names();
    for (const auto& name : config.names) {
      std::cout << "State interface config name: " << name << std::endl;
    }
  }
  config.names.push_back(arm_id_ + "/robot_time");
  return config;
}

controller_interface::return_type JoystickCartesianPoseExampleController::update(const rclcpp::Time& /*time*/,
                                                                                 const rclcpp::Duration& /*period*/) {
  if (initialization_flag_) {
    std::tie(orientation_, position_) = franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    if (!state_interfaces_.empty()) {
      initial_robot_time_ = state_interfaces_.back().get_value();
    }
    elapsed_time_ = 0.0;
    initialization_flag_ = false;
  } else {
    if (!state_interfaces_.empty()) {
      robot_time_ = state_interfaces_.back().get_value();
      elapsed_time_ = robot_time_ - initial_robot_time_;
    }
  }

  // Use current pose from joystick control
  position_(0) = current_pose_.pose.position.x;
  position_(1) = current_pose_.pose.position.y;
  position_(2) = current_pose_.pose.position.z;
  
  orientation_.w() = current_pose_.pose.orientation.w;
  orientation_.x() = current_pose_.pose.orientation.x;
  orientation_.y() = current_pose_.pose.orientation.y;
  orientation_.z() = current_pose_.pose.orientation.z;
  orientation_.normalize();

  if (franka_cartesian_pose_ && franka_cartesian_pose_->setCommand(orientation_, position_)) {
    return controller_interface::return_type::OK;
  }
  RCLCPP_FATAL(get_node()->get_logger(),
               "Set command failed. Did you activate the elbow command interface?");
  return controller_interface::return_type::ERROR;
}

CallbackReturn JoystickCartesianPoseExampleController::on_init() {
  franka_cartesian_pose_ = std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(k_elbow_activated_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn JoystickCartesianPoseExampleController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  // 尝试设置默认碰撞行为（非致命）
  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  if (client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_INFO(get_node()->get_logger(), "Collision behavior service available, skipping detailed setup for now.");
    // 如果你想恢复原始逻辑，在这里填入正确的 request 构造并发送。
    // 例如（伪代码，需替换为实际的 helper）：
    // auto request = DefaultRobotBehaviorUtils::getDefaultCollisionBehaviorRequest();
    // auto future_result = client->async_send_request(request);
    // if (rclcpp::spin_until_future_complete(get_node()->get_node_base_interface(), future_result,
    //                                        std::chrono::seconds(2)) != rclcpp::FutureReturnCode::SUCCESS) {
    //   RCLCPP_WARN(get_node()->get_logger(), "Failed to call collision behavior service.");
    // } else {
    //   RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
    // }
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "Collision behavior service not available; continuing without setting it.");
  }

  // 获取 robot_description
  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  if (!parameters_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_state_publisher parameters service unavailable.");
    return CallbackReturn::ERROR;
  }

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  // Initialize current pose
  current_pose_.header.frame_id = "base_link";
  current_pose_.pose.orientation.w = 1.0;
  last_joy_time_ = get_node()->get_clock()->now();

  // Subscribe to joystick input
  joy_subscriber_ = get_node()->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", rclcpp::SystemDefaultsQoS(),
      std::bind(&JoystickCartesianPoseExampleController::joyCallback, this,
                std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(), "Joystick Cartesian Pose Controller configured successfully");

  return CallbackReturn::SUCCESS;
}

// CallbackReturn JoystickCartesianPoseExampleController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
//   // 设置默认碰撞行为
//   auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
//       "service_server/set_full_collision_behavior");
//   if (!client->wait_for_service(std::chrono::seconds(5))) {
//     RCLCPP_FATAL(get_node()->get_logger(), "Collision behavior service not available.");
//     return CallbackReturn::ERROR;
//   }
//   auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
//   auto future_result = client->async_send_request(request);
//   if (rclcpp::spin_until_future_complete(get_node()->get_node_base_interface(), future_result,
//                                          std::chrono::seconds(2))
//       != rclcpp::FutureReturnCode::SUCCESS) {
//     RCLCPP_FATAL(get_node()->get_logger(), "Failed to call collision behavior service.");
//     return CallbackReturn::ERROR;
//   }
//   // 这里根据返回类型检查成功与否，略

//   // 获取 robot_description
//   auto parameters_client =
//       std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
//   if (!parameters_client->wait_for_service(std::chrono::seconds(5))) {
//     RCLCPP_ERROR(get_node()->get_logger(), "robot_state_publisher parameters service unavailable.");
//     return CallbackReturn::ERROR;
//   }

//   auto future = parameters_client->get_parameters({"robot_description"});
//   auto result = future.get();
//   if (!result.empty()) {
//     robot_description_ = result[0].value_to_string();
//   } else {
//     RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
//   }

//   arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

//   // 订阅外部 pose 命令
//   cartesian_pose_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
//       "/cartesian_pose_command", rclcpp::SystemDefaultsQoS(),
//       std::bind(&JoystickCartesianPoseExampleController::cartesianPoseCallback, this,
//                 std::placeholders::_1));

//   return CallbackReturn::SUCCESS;
// }

CallbackReturn JoystickCartesianPoseExampleController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  if (franka_cartesian_pose_) {
    franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
    franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JoystickCartesianPoseExampleController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  if (franka_cartesian_pose_) {
    franka_cartesian_pose_->release_interfaces();
  }
  return CallbackReturn::SUCCESS;
}

void JoystickCartesianPoseExampleController::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  // Joystick Mapping:
  // Left Stick Y (axes[1]) -> X-axis movement
  // Left Stick X (axes[0]) -> Y-axis movement  
  // Right Stick Y (axes[4]) -> Z-axis movement
  // Right Stick X (axes[3]) -> Yaw rotation

  // Get the time since the last update to calculate incremental changes
  rclcpp::Time now = get_node()->get_clock()->now();
  double dt = (now - last_joy_time_).seconds();
  last_joy_time_ = now;

  // Calculate incremental position changes based on joystick axes
  double delta_x = 0.0;
  if (std::abs(msg->axes[1]) > DEADZONE) {
    delta_x = msg->axes[1] * TRANSLATION_SPEED * dt;
  }

  double delta_y = 0.0;
  if (std::abs(msg->axes[0]) > DEADZONE) {
    delta_y = msg->axes[0] * TRANSLATION_SPEED * dt;
  }

  double delta_z = 0.0;
  if (std::abs(msg->axes[4]) > DEADZONE) {
    delta_z = msg->axes[4] * TRANSLATION_SPEED * dt;
  }

  // Calculate incremental orientation changes (Yaw only)
  double delta_yaw = 0.0;
  if (std::abs(msg->axes[3]) > DEADZONE) {
    delta_yaw = msg->axes[3] * ROTATION_SPEED * dt;
  }
  
  // Update the current pose based on the deltas
  current_pose_.pose.position.x += delta_x;
  current_pose_.pose.position.y += delta_y;
  current_pose_.pose.position.z += delta_z;

  // Convert the current pose orientation to an Eigen quaternion for manipulation
  Eigen::Quaterniond orientation_eigen(current_pose_.pose.orientation.w,
                                        current_pose_.pose.orientation.x,
                                        current_pose_.pose.orientation.y,
                                        current_pose_.pose.orientation.z);
  
  // Create a new rotation quaternion for the delta yaw
  Eigen::Quaterniond rotation_delta(
      Eigen::AngleAxisd(delta_yaw, Eigen::Vector3d::UnitZ()));

  // Apply the new rotation and normalize the result
  orientation_eigen = orientation_eigen * rotation_delta;
  orientation_eigen.normalize();

  // Update the pose message with the new orientation
  current_pose_.pose.orientation.w = orientation_eigen.w();
  current_pose_.pose.orientation.x = orientation_eigen.x();
  current_pose_.pose.orientation.y = orientation_eigen.y();
  current_pose_.pose.orientation.z = orientation_eigen.z();
  
  // Update the header timestamp
  current_pose_.header.stamp = now;
}

}  // namespace franka_example_controllers

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JoystickCartesianPoseExampleController,
                       controller_interface::ControllerInterface)

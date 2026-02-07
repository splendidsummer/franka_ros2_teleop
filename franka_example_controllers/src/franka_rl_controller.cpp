// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_example_controllers/robot_description.hpp>
#include <franka_example_controllers/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
JointPositionExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;  // In update()

// Currently, this controller does not publish joint states. 
//   std_msgs::msg::Float64MultiArray msg;
//   msg.data.resize(num_joints);
//   for (int i = 0; i < num_joints; ++i) {
//     msg.data[i] = state_interfaces_[i].get_value();
//   }
//   joint_state_pub_->publish(msg);
// }

controller_interface::InterfaceConfiguration
JointPositionExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  std::cout << "the arm_id_ is: " << arm_id_ << std::endl;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }

  // add the robot time interface
  if (!is_gazebo_) {
    config.names.push_back(arm_id_ + "/robot_time");
  }

  return config;/home/summer/Downloads/franka_ros (1)/src/franka_sideview_cameras
}

controller_interface::return_type JointPositionExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (initialization_flag_) {
    for (int i = 0; i < num_joints; ++i) {
      initial_q_.at(i) = state_interfaces_[i].get_value();
    }

    auto msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
    msg->data = initial_q_;
    static rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr init_joint_publisher = 
        get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("joint_initial_state", 10);
    publisher->publish(*msg);
    RCLCPP_INFO(get_node()->get_logger(), "Publishing initial joint positions: [%f, %f, %f, %f, %f, %f, %f]",
              initial_q_[0], initial_q_[1], initial_q_[2], initial_q_[3], initial_q_[4], initial_q_[5], initial_q_[6]);

    initialization_flag_ = false;
    if (!is_gazebo_) {
      initial_robot_time_ = state_interfaces_.back().get_value();
    }
    elapsed_time_ = 0.0;


  } else {
    if (!is_gazebo_) {
      robot_time_ = state_interfaces_.back().get_value();
      elapsed_time_ = robot_time_ - initial_robot_time_;
    } else {
      elapsed_time_ += trajectory_period_;
    }
  }

  // Here is designed to get the end effector position and orientation from parameters
  // Which can be set in the launch file or through command line
  /**
   * \todo: I am questioning whether parameters should be set in the launch file?  or through command line?.
   * This is because the end effector position and orientation are not fixed,
   * and they can be changed dynamically during the execution of the controller.
   * If we set them in the launch file, we need to restart the controller to change them.
   * If we set them through command line, we can change them dynamically without restarting the controller.
   * So I think it is better to set them through command line.
   * But I am not sure if it is a good practice to set parameters through command line
   * in ROS 2. I need to check the documentation and see if it is a common practice.
  
   */
  
  double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_)) * 0.2;

  for (int i = 0; i < num_joints; ++i) {
    if (i == 4) {
      command_interfaces_[i].set_value(initial_q_.at(i) - delta_angle);
    } else {
      command_interfaces_[i].set_value(initial_q_.at(i) + delta_angle);
    }
  }

  return controller_interface::return_type::OK;
}

CallbackReturn JointPositionExampleController::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
    end_effector_position_.resize(3, 0.0);
    end_effector_orientation_.resize(4, 0.0);

    // Optionally, get user input from parameters
    // Declare parameters with default values if they are not already declared
    // Prompt user for end effector position
    std::vector<double> user_desired_position(3, 0.0);
    std::vector<double> user_desired_orientation(4, 0.0);
    
    std::cout << "Enter end effector position (x y z): ";
    for (int i = 0; i < 3; ++i) {
      if (i == 0) {
        std::cout << " (range: " << EE_X_MIN << " to " << EE_X_MAX << ")";
      } else if (i == 1) {
        std::cout << " (range: " << EE_Y_MIN << " to " << EE_Y_MAX << ")";
      } else if (i == 2) {
        std::cout << " (range: " << EE_Z_MIN << " to " << EE_Z_MAX << ")";
      }
      std::cin >> user_position[i];
      if ((i == 0 && (user_position[i] < EE_X_MIN || user_position[i] > EE_X_MAX)) ||
          (i == 1 && (user_position[i] < EE_Y_MIN || user_position[i] > EE_Y_MAX)) ||
          (i == 2 && (user_position[i] < EE_Z_MIN || user_position[i] > EE_Z_MAX))) {
        std::cerr << "Error: Input value out of range for dimension " << i << std::endl;
        throw std::runtime_error("End effector position input out of range.");
      }
    }


    std::cout << "Enter end effector orientation (x y z w): ";
    for (int i = 0; i < 4; ++i) {
      std::cin >> user_orientation[i];
    }

    // 假设 user_orientation 是 std::vector<double>，长度为4
    double norm = std::sqrt(
        user_orientation[0]*user_orientation[0] +
        user_orientation[1]*user_orientation[1] +
        user_orientation[2]*user_orientation[2] +
        user_orientation[3]*user_orientation[3]);
    for (int i = 0; i < 4; ++i) {
        user_orientation[i] /= norm;
    }

    // Declare and set parameters
    if (!get_node()->has_parameter("end_effector_position")) {
      get_node()->declare_parameter<std::vector<double>>("end_effector_position", user_position);
    } else {
      get_node()->set_parameter(rclcpp::Parameter("end_effector_position", user_position));
    }
    if (!get_node()->has_parameter("end_effector_orientation")) {
      get_node()->declare_parameter<std::vector<double>>("end_effector_orientation", user_orientation);
    } else {
      get_node()->set_parameter(rclcpp::Parameter("end_effector_orientation", user_orientation));
    }

    if (get_node()->has_parameter("end_effector_position")) {
      auto pos_param = get_node()->get_parameter("end_effector_position").as_double_array();
      if (pos_param.size() == 3) {
        end_effector_position_ = pos_param;
      }
    }
    if (get_node()->has_parameter("end_effector_orientation")) {
      auto ori_param = get_node()->get_parameter("end_effector_orientation").as_double_array();
      if (ori_param.size() == 4) {
        end_effector_orientation_ = ori_param;
      }
    }
    
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  is_gazebo_ = get_node()->get_parameter("gazebo").as_bool();

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointPositionExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  return CallbackReturn::SUCCESS;
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerInterface)

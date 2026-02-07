// 版权所有 (c) 2023 Franka Robotics GmbH
//
// 根据 Apache License, Version 2.0（“许可证”）授权；
// 除非遵守许可证，否则您不得使用此文件。
// 您可以在以下网址获取许可证副本：
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// 除非适用法律要求或书面同意，否则根据许可证分发的软件
// 按“原样”分发，无任何明示或暗示的担保或条件。
// 有关许可证下权限和限制的具体语言，请参见许可证。

#include <franka_example_controllers/joint_position_example_controller.hpp> // 引入头文件
#include <franka_example_controllers/robot_utils.hpp> // 引入工具头文件
#include <cassert>    // 断言库
#include <cmath>      // 数学库
#include <exception>  // 异常处理库
#include <string>     // 字符串库

#include <Eigen/Eigen> // Eigen线性代数库

namespace franka_example_controllers { // 命名空间开始
 
controller_interface::InterfaceConfiguration
JointPositionExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config; // 创建配置对象
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL; // 设置类型为单独接口
  for (int i = 1; i <= num_joints; ++i) { // 遍历每个关节
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position"); // 添加每个关节的位置接口名
  }
  return config; // 返回配置
}

// 定义状态接口配置函数


  /**
   * \note: comment example: 
   * @remark: state_interfaces_ is defined in controller_interface::ControllerInterface? 
   * \name:  filling the context later 
   * @param[in] command_elbow_active insert true to activate the elbow commanding together with the
   * cartesian velocity.
   * 
   */

controller_interface::InterfaceConfiguration
JointPositionExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config; // 创建配置对象
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL; // 设置类型为单独接口

  for (int i = 1; i <= num_joints; ++i) { // 遍历每个关节
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position"); // 添加每个关节的位置接口名
  }

  for (const auto& name : config.names) {
    std::cout << "Config name: " << name << std::endl;
  }
  // 如果不是仿真环境，添加机器人时间接口
  if (!is_gazebo_) {
    config.names.push_back(arm_id_ + "/robot_time");
  }

  return config; // 返回配置
}

// 控制器主循环更新函数
/**
 * @attention
 * @param
 * @brief
 */
controller_interface::return_type JointPositionExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (initialization_flag_) { // 如果是初始化阶段
    for (int i = 0; i < num_joints; ++i) { // 遍历每个关节
      // state_interfaces 
      initial_q_.at(i) = state_interfaces_[i].get_value(); // 记录初始关节位置
    }
    initialization_flag_ = false; // 取消初始化标志
    if (!is_gazebo_) { // 如果不是仿真环境
      initial_robot_time_ = state_interfaces_.back().get_value(); // 记录初始机器人时间
    }
    elapsed_time_ = 0.0; // 重置已用时间
  } else {
    if (!is_gazebo_) { // 如果不是仿真环境
      robot_time_ = state_interfaces_.back().get_value(); // 获取当前机器人时间
      elapsed_time_ = robot_time_ - initial_robot_time_; // 计算已用时间
    } else {
      elapsed_time_ += trajectory_period_; // 仿真环境下，已用时间累加
    }
  }

  // 计算关节角度变化量
  double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_)) * 0.2;

  for (int i = 0; i < num_joints; ++i) { // 遍历每个关节
    if (i == 4) { // 如果是第5个关节（下标从0开始）
      command_interfaces_[i].set_value(initial_q_.at(i) - delta_angle); // 该关节反向运动
    } else {
      command_interfaces_[i].set_value(initial_q_.at(i) + delta_angle); // 其他关节正向运动
    }
  }

  if (!is_gazebo_) { // 如果不是仿真环境
    command_interfaces_.back().set_value(robot_time_ + elapsed_time_); // 设置机器人时间
  }
                
  return controller_interface::return_type::OK; // 返回OK
}

// 控制器初始化回调
CallbackReturn JointPositionExampleController::on_init() {
  try {
    auto_declare<bool>("gazebo", false); // 声明gazebo参数，默认false
    auto_declare<std::string>("robot_description", ""); // 声明robot_description参数
  } catch (const std::exception& e) { // 捕获异常
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what()); // 打印异常信息
    return CallbackReturn::ERROR; // 返回错误
  }
  return CallbackReturn::SUCCESS; // 返回成功
}

// 控制器配置回调
CallbackReturn JointPositionExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  is_gazebo_ = get_node()->get_parameter("gazebo").as_bool(); // 获取gazebo参数

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher"); // 创建参数客户端
  parameters_client->wait_for_service(); // 等待服务可用

  auto future = parameters_client->get_parameters({"robot_description"}); // 获取robot_description参数
  auto result = future.get(); // 获取结果
  if (!result.empty()) { // 如果获取成功
    robot_description_ = result[0].value_to_string(); // 保存机器人描述
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter."); // 打印错误日志
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger()); // 解析机器人名称

  return CallbackReturn::SUCCESS; // 返回成功
}

// 控制器激活回调
CallbackReturn JointPositionExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true; // 设置初始化标志
  elapsed_time_ = 0.0; // 重置已用时间
  return CallbackReturn::SUCCESS; // 返回成功
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp" // 插件宏定义头文件
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerInterface) // 导出插件类


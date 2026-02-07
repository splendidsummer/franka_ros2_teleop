// Copyright 2024 Google LLC
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

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Define a scaling factor for movement speed.
const double TRANSLATION_SPEED = 0.1; // meters per second
const double ROTATION_SPEED = 0.5;    // radians per second

/**
 * @class JoystickPosePublisher
 * @brief A simple ROS 2 node to read joystick input and publish a Cartesian pose command.
 * * This node subscribes to the /joy topic and maps specific joystick axes to control
 * the X, Y, Z position and Yaw orientation of a robot. The resulting pose is published
 * on the /cartesian_pose_command topic.
 */
class JoystickPosePublisher : public rclcpp::Node {
public:
    JoystickPosePublisher() : Node("joystick_pose_publisher") {
        RCLCPP_INFO(this->get_logger(), "Starting Joystick Pose Publisher Node");

        // Create the publisher for the Cartesian pose command.
        // It publishes a PoseStamped message to the "cartesian_pose_command" topic.
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "cartesian_pose_command", 10);

        // Create the subscriber for the joystick message.
        // It subscribes to the "joy" topic and calls the joy_callback function.
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoystickPosePublisher::joy_callback, this, std::placeholders::_1));
        
        // Initialize the current pose to the identity pose.
        // Set the reference frame
        current_pose_.header.frame_id = "base_link"; 
        // Identity quaternion
        current_pose_.pose.orientation.w = 1.0; 

        // 这行保证 last_joy_time_ 使用和 get_clock() 相同的时钟来源
        last_joy_time_ = this->get_clock()->now();
    }

private:
    /**
     * @brief Callback function for the joystick subscriber.
     * @param msg The received joystick message.
     * * This function is triggered whenever a new joystick message is received.
     * It reads the joystick axes and uses them to calculate the new pose.
     */
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // --- Joystick Mapping: ---
        // Left Stick Y (axes[1]) -> X-axis movement
        // Left Stick X (axes[0]) -> Y-axis movement
        // Right Stick Y (axes[4]) -> Z-axis movement
        // Right Stick X (axes[3]) -> Yaw rotation

        // A small threshold to prevent drift from a slightly off-center joystick.
        const double deadzone = 0.10;

        // Get the time since the last update to calculate incremental changes.
        // This makes the movement rate independent of the callback frequency.
        rclcpp::Time now = this->get_clock()->now();
        double dt = (now - last_joy_time_).seconds();
        last_joy_time_ = now;

        // Calculate incremental position changes based on joystick axes.
        double delta_x = 0.0;
        if (std::abs(msg->axes[1]) > deadzone) {
            delta_x = msg->axes[1] * TRANSLATION_SPEED * dt;
        }

        double delta_y = 0.0;
        if (std::abs(msg->axes[0]) > deadzone) {
            delta_y = msg->axes[0] * TRANSLATION_SPEED * dt;
        }

        double delta_z = 0.0;
        if (std::abs(msg->axes[4]) > deadzone) {
            delta_z = msg->axes[4] * TRANSLATION_SPEED * dt;
        }

        // Calculate incremental orientation changes. Pitch and Roll are not controlled.
        double delta_yaw = 0.0;
        if (std::abs(msg->axes[3]) > deadzone) {
            delta_yaw = msg->axes[3] * ROTATION_SPEED * dt;
        }
        
        // Update the current pose based on the deltas.
        current_pose_.pose.position.x += delta_x;
        current_pose_.pose.position.y += delta_y;
        current_pose_.pose.position.z += delta_z;

        // Convert the current pose orientation to an Eigen quaternion for manipulation.
        Eigen::Quaterniond orientation_eigen(current_pose_.pose.orientation.w,
                                              current_pose_.pose.orientation.x,
                                              current_pose_.pose.orientation.y,
                                              current_pose_.pose.orientation.z);
        
        // Create a new rotation quaternion for the delta yaw.
        Eigen::Quaterniond rotation_delta(
            Eigen::AngleAxisd(delta_yaw, Eigen::Vector3d::UnitZ()));

        // Apply the new rotation and normalize the result.
        orientation_eigen = orientation_eigen * rotation_delta;
        orientation_eigen.normalize();

        // Update the pose message with the new orientation.
        current_pose_.pose.orientation.w = orientation_eigen.w();
        current_pose_.pose.orientation.x = orientation_eigen.x();
        current_pose_.pose.orientation.y = orientation_eigen.y();
        current_pose_.pose.orientation.z = orientation_eigen.z();
        
        // Update the header timestamp and publish the new pose.
        current_pose_.header.stamp = now;
        pose_publisher_->publish(current_pose_);

    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    geometry_msgs::msg::PoseStamped current_pose_;

    // 移除“= rclcpp::Time(0)”的默认初始化
    rclcpp::Time last_joy_time_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

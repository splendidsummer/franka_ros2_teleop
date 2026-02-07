import torch
import rclpy
from rclpy.node import Node
from franka_msgs.msg import FrankaRobotState
import numpy as np
from pathlib import Path
from std_msgs.msg import Float64MultiArray


##########################################################################
# Utils functions  
##########################################################################
def load_network():
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    current_file_path = Path(__file__).resolve()
    parent_folder = current_file_path.parent
    pth_path = parent_folder / 'saved_models'/ 'xxxx.pth'

    model = torch.load(pth_path, map_location=device)
    model.eval()
    return model 


def extract_observations(msg):
    # Extract relevant observations from the FrankaRobotState message
    observations = {
        'joint_positions': msg.measured_joint_state.position,
        'joint_velocities': msg.measured_joint_state.velocity,
        # 'external_torques': msg.tau_ext_hat_filtered.effort,
        # 'robot_mode': msg.robot_mode,
        # # Add more observations as needed
    }
    return observations


def convert_batch_tensor(data):
    data = np.array(data, dtype=np.float32)
    return torch.tensor(data, dtype=torch.float32).unsqueeze(0)


def process_data(observations, initialized_joint_positions):
    # Retrieve the initialized joint positions if needed
    # For example, you might store them as a class attribute when the node starts
    # Here, just as a placeholder, you could use a fixed value or extract from observations
    # Example: initialized_joint_positions = self.initialized_joint_positions if hasattr(self, 'initialized_joint_positions') else observations['joint_positions']
    # Convert observations to a tensor suitable for the model
    # Example assumes observations is a dict of numpy arrays or lists
    joint_pos_rel = observations['joint_positions']- initialized_joint_positions
    obs_array = np.concatenate([
        np.array(joint_pos_rel, dtype=np.float32),
        np.array(observations['joint_velocities']),
        # np.array(observations['external_torques']),
        # np.array([observations['robot_mode']])
    ])
    
    return obs_array

class PolicyNode(Node):
    def __init__(self):
        super().__init__('joint_initial_state')
        # Subscribe to multiple FrankaRobotState topics
        self.franka_state_sub = self.create_subscription(
            FrankaRobotState,
            'franka_robot_state',
            self.listener_callback,
            10
        )
        self.joint_init_state_sub = self.create_subscription(
            Float64MultiArray,
            'joint_initial_state',
            self.listener_callback,
            10
        )
        
        self.action_publisher = self.create_publisher(  # 2. Create publisher
            Float64MultiArray,
            'rl_policy_action',
            10
        )

        # We are supposing the action is pre-prosessed by the RL controller after 
        # the policy action is received by franka_rl_controller 
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'rl_control_action',
            self.listener_callback,
            10
        )
        
        self.get_logger().info('PolicyNode has been started.')
        
        self.policy_network = load_network()
        self.get_logger().info('PolicyNode has been initialized.')
        
        self.ee_pos = self.declare_parameter('end_effector_position', [0.0, 0.0, 0.0]).get_parameter_value().double_array_value
        self.ee_ori = self.declare_parameter('end_effector_orientation', [0.0, 0.0, 0.0, 1.0]).get_parameter_value().double_array_value

        self.get_logger().info(f'End effector position: {self.ee_pos}')
        self.get_logger().info(f'End effector orientation: {self.ee_ori}')
        

    def listener_callback(self, msg):
        
        try:
            if isinstance(msg, FrankaRobotState):
                self.observations = extract_observations(msg)
            elif isinstance(msg, Float64MultiArray):
                topic = getattr(msg, '_topic_name', None)
                if topic == 'joint_initial_state':
                    self.initial_joint_state = msg.data
                    self.get_logger().info(f"Received initial joint state: {self.initial_joint_state}")
                elif topic == 'rl_control_action':
                    self.last_action = msg.data
                    self.get_logger().info(f"Received last action: {self.last_action}")
            else:
                raise TypeError(f"Received unknown message type: {type(msg)}")
            
        except Exception as error:
            self.get_logger().error(f"Error in listener_callback: {error}")
            
        self.obs_tensor = convert_batch_tensor(process_data(self.observations))
        self.ee_pos_tensor = convert_batch_tensor(self.ee_pos)
        self.ee_ori_tensor = convert_batch_tensor(self.ee_ori)
        self.last_action_tensor = convert_batch_tensor(self.last_action) 
        self.input_tensor = torch.cat((self.obs_tensor, self.ee_pos_tensor, self.ee_ori_tensor, self.last_action_tensor), dim=1)
        
        self.get_logger().info(f"Input tensor shape: {self.input_tensor.shape}")
        # Ensure the input tensor is on the same device as the model
        self.input_tensor = self.input_tensor.to(self.policy_network.device)
        
        # Forward pass through the policy network
        with torch.no_grad():
            action = self.policy_network(self.input_tensor)
            
        # Convert the action tensor to a Float64MultiArray message
        rl_action_msg = Float64MultiArray()
        rl_action_msg.data = action.cpu().numpy().flatten().tolist()
        
        self.action_publisher.publish(rl_action_msg)
#         self.get_logger().info(f"Published action: {action_msg.data}")    

def main(args=None):
    rclpy.init(args=args)
    policy_node = PolicyNode()
    rclpy.spin(policy_node)
    policy_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
###########################################################################################
# Note: C++ code is not used in this file, but it is included in the project.
#       The C++ code is used in the franka_rl_controller node to process the action
#       and send it to the franka_ros2_control node.
#       The C++ code is not used in this file, but it is included in the project.
#       The C++ code is used in the franka_rl_controller node to process the    
#       action and send it to the franka_ros2_control node.
#       The C++ code is not used in this file, but it is included in the project.
#       The C++ code is used in the franka_rl_controller node to process the
#       action and send it to the franka_ros2_control node.
#       The C++ code is not used in this file, but it is included in the project.
#       The C++ code is used in the franka_rl_controller node to process the
#       action and send it to the franka_ros2_control node.
###########################################################################################

#include <rclcpp/rclcpp.hpp>
#include <franka_msgs/msg/franka_robot_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <torch/script.h>  // LibTorch
#include <vector>
#include <memory>

# class PolicyNode : public rclcpp::Node {
# public:
#   PolicyNode() : Node("joint_initial_state") {
#     // 参数声明
#     this->declare_parameter<std::vector<double>>("end_effector_position", {0.0, 0.0, 0.0});
#     this->declare_parameter<std::vector<double>>("end_effector_orientation", {0.0, 0.0, 0.0, 1.0});
#     ee_pos_ = this->get_parameter("end_effector_position").as_double_array();
#     ee_ori_ = this->get_parameter("end_effector_orientation").as_double_array();

#     // 订阅
#     franka_state_sub_ = this->create_subscription<franka_msgs::msg::FrankaRobotState>(
#       "franka_robot_state", 10,
#       std::bind(&PolicyNode::listener_callback, this, std::placeholders::_1));
#     joint_init_state_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
#       "joint_initial_state", 10,
#       std::bind(&PolicyNode::listener_callback, this, std::placeholders::_1));
#     rl_control_action_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
#       "rl_control_action", 10,
#       std::bind(&PolicyNode::listener_callback, this, std::placeholders::_1));

#     // 发布
#     action_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("rl_policy_action", 10);

#     // 加载模型
#     torch::Device device(torch::kCPU);
#     try {
#       module_ = torch::jit::load("PATH_TO_YOUR_MODEL.pt", device);
#     } catch (const c10::Error& e) {
#       RCLCPP_ERROR(this->get_logger(), "Error loading the model");
#     }
#   }

# private:
#   // 成员变量
#   std::vector<double> ee_pos_;
#   std::vector<double> ee_ori_;
#   std::vector<double> initial_joint_state_;
#   std::vector<double> last_action_;
#   std::map<std::string, double> observations_;
#   torch::jit::script::Module module_;

#   rclcpp::Subscription<franka_msgs::msg::FrankaRobotState>::SharedPtr franka_state_sub_;
#   rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_init_state_sub_;
#   rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rl_control_action_sub_;
#   rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr action_pub_;

#   // 回调函数重载
#   void listener_callback(const franka_msgs::msg::FrankaRobotState::SharedPtr msg) {
#     // 提取观测量
#     // observations_["joint_positions"] = ...;
#     // observations_["joint_velocities"] = ...;
#     // ...
#     // 你可以在这里保存观测量
#   }
#   void listener_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
#     // 判断话题名，区分 joint_initial_state 和 rl_control_action
#     // 保存数据到 initial_joint_state_ 或 last_action_
#     // ...
#     // 当数据齐全时，拼接输入，送入网络
#     // torch::Tensor input = ...;
#     // auto output = module_.forward({input}).toTensor();
#     // 发布动作
#     // std_msgs::msg::Float64MultiArray action_msg;
#     // action_msg.data = ...;
#     // action_pub_->publish(action_msg);
#   }
# };

# int main(int argc, char* argv[]) {
#   rclcpp::init(argc, argv);
#   auto node = std::make_shared<PolicyNode>();
#   rclcpp::spin(node);
#   rclcpp::shutdown();
#   return 0;
# }
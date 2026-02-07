# Franka ROS2 工作空间

[![CI](https://github.com/frankarobotics/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankarobotics/franka_ros2/actions/workflows/ci.yml)

> **注意：** _franka_ros2_ 官方不支持 Windows。

## 目录
- [概述](#概述)
- [注意事项](#注意事项)
- [架构](#架构)
  - [核心组件](#核心组件)
  - [关键架构特性](#关键架构特性)
- [软件包](#软件包)
  - [核心基础设施](#核心基础设施)
  - [控制系统](#控制系统)
  - [仿真](#仿真)
  - [机器人描述与规划](#机器人描述与规划)
  - [研究与扩展](#研究与扩展)
- [可用控制器](#可用控制器)
  - [关节空间控制](#关节空间控制)
  - [笛卡尔空间控制](#笛卡尔空间控制)
  - [专用控制器](#专用控制器)
- [快速开始](#快速开始)
  - [先决条件](#先决条件)
  - [安装](#安装)
  - [快速启动示例](#快速启动示例)
- [配置](#配置)
  - [机器人配置](#机器人配置)
  - [控制器配置](#控制器配置)
  - [命名空间支持](#命名空间支持)
- [开发](#开发)
  - [创建自定义控制器](#创建自定义控制器)
  - [硬件接口扩展](#硬件接口扩展)
  - [研究集成](#研究集成)
- [测试](#测试)
  - [单元测试](#单元测试)
  - [集成测试](#集成测试)
  - [仿真测试](#仿真测试)
- [故障排除](#故障排除)
  - [常见问题](#常见问题)
  - [实时配置](#实时配置)
- [研究应用](#研究应用)
  - [强化学习](#强化学习)
  - [计算机视觉](#计算机视觉)
  - [运动规划](#运动规划)
- [贡献指南](#贡献指南)
- [许可证](#许可证)
- [资源](#资源)
- [支持](#支持)
- [联系方式](#联系方式)

一个全面的 ROS 2 Humble 工作空间，用于 Franka Emika 研究机器人，为 Franka FR3 机器人提供硬件接口、控制器、仿真和研究工具。

## 概述

该工作空间通过 `libfranka` 库将 Franka 机器人与 ROS 2 集成，为机器人研究和开发提供完整的生态系统。系统支持真实机器人控制、仿真、多种控制策略以及如强化学习和计算机视觉等研究扩展。

## 注意事项

该软件包正在快速开发中。用户应预期会有破坏性更改，并鼓励通过 [GitHub Issues 页面](https://github.com/frankarobotics/franka_ros2/issues) 报告任何错误。

## 架构

### 核心组件

工作空间遵循基于 `ros2_control` 的模块化 ROS 2 架构：

```
┌─────────────────────────────────────────────────────────┐
│                   应用层                                 │
│  (示例控制器、研究包、MoveIt2)                          │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│                   控制层                                 │
│          (ROS 2 控制框架 - controller_manager)          │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│                 硬件抽象层                               │
│      (franka_hardware - SystemInterface 实现)           │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│                  通信层                                 │
│                (libfranka - FCI 协议)                   │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│                   物理/仿真层                            │
│          (真实 Franka 机器人 / Gazebo 仿真)             │
└─────────────────────────────────────────────────────────┘
```

### 关键架构特性

- **命名空间支持**：为多机器人设置提供完整的命名空间隔离
- **硬件抽象**：真实硬件和仿真的统一接口
- **基于插件的控制器**：通过 `pluginlib` 实现可扩展的控制器架构
- **语义组件**：机器人状态和命令的类型化接口
- **实时能力**：设计用于实时控制，需正确配置内核

## 软件包

### 核心基础设施

| 软件包 | 目的 | 关键组件 |
|---------|---------|----------------|
| `franka_hardware` | 硬件接口实现 | `FrankaHardwareInterface`, `Robot`, `FrankaExecutor` |
| `franka_msgs` | 自定义消息、服务、动作 | `FrankaRobotState`、碰撞服务、抓取动作 |
| `franka_semantic_components` | ROS 2 控制的语义接口 | `FrankaRobotState` 组件 |

### 控制系统

| 软件包 | 目的 | 特性 |
|---------|---------|----------|
| `franka_example_controllers` | 示例控制实现 | 15+ 个关节/笛卡尔空间控制器 |
| `franka_robot_state_broadcaster` | 状态信息发布器 | 可配置的锁管理 |
| `franka_bringup` | 启动文件和配置 | 命名空间支持、多机器人、虚拟硬件 |

### 仿真

| 软件包 | 目的 | 特性 |
|---------|---------|----------|
| `franka_gazebo` | Gazebo/Ingition 集成 | 关节位置/速度/阻抗控制示例 |
| `franka_ign_ros2_control` | Ignition Gazebo ROS 2 控制插件 | 扭矩控制支持 |

### 机器人描述与规划

| 软件包 | 目的 | 特性 |
|---------|---------|----------|
| `franka_description` | URDF 模型和网格 | FR3 机器人描述 |
| `franka_fr3_moveit_config` | MoveIt2 配置 | OMPL 规划器、运动学、可视化 |
| `franka_gripper` | 夹爪控制 | Python/C++ 实现 |

### 研究与扩展

| 软件包 | 目的 | 特性 |
|---------|---------|----------|
| `franka_rl_control` | 强化学习集成 | PyTorch 策略执行 |
| `franka_cameras` | RealSense 相机集成 | 图像传输、CV 桥接 |
| `franka_sideview_cameras` | 双相机发布器 | PyRealSense2、OpenCV |
| `franka_joystick` | 遥操作接口 | 游戏控制器到笛卡尔位姿 |
| `franka_vla_inference_service` | 视觉-语言-动作推理 | (开发中) |

## 可用控制器

系统为不同的控制策略提供全面的控制器集合：

### 关节空间控制
- **关节位置控制**：基于位置的关节控制
- **关节速度控制**：基于速度的关节控制
- **关节阻抗控制**：基于扭矩的阻抗控制
- **带逆运动学的关节阻抗控制**：带逆运动学的阻抗控制

### 笛卡尔空间控制
- **笛卡尔位姿控制**：末端执行器位置/姿态控制
- **笛卡尔速度控制**：末端执行器速度控制
- **笛卡尔肘部控制**：带肘部配置的笛卡尔控制
- **笛卡尔姿态控制**：仅姿态控制

### 专用控制器
- **重力补偿**：零力重力补偿
- **移动到起始位置**：平滑移动到起始位置
- **基于模型的控制**：基于模型的扭矩控制
- **夹爪控制**：平行夹爪控制
- **动态波浪**：波浪模式演示
- **游戏手柄笛卡尔位姿**：遥操作控制器

## 快速开始

### 先决条件
- **ROS 2 Humble** (桌面版或基础版安装)
- **实时内核** (用于真实机器人操作)
- **Docker** (可选，用于容器化开发)

### 安装

```bash
# 创建工作空间
mkdir -p ~/franka_ros2_ws/src
cd ~/franka_ros2_ws

# 克隆仓库
git clone https://github.com/frankaemika/franka_ros2.git src

# 导入依赖
vcs import src < src/franka.repos --recursive --skip-existing

# 安装 ROS 依赖
rosdep install --from-paths src --ignore-src --rosdistro humble -y

# 构建工作空间
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 设置工作空间环境
source install/setup.bash
```

### 快速启动示例

#### 仿真 (无需真实机器人)
```bash
# 使用虚拟硬件启动 MoveIt2
ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true

# 运行特定控制器
ros2 launch franka_bringup example.launch.py controller_name:=joint_position_example_controller
```

#### Gazebo 仿真
```bash
# 在 Gazebo 中可视化机器人
ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py

# 关节位置控制示例
ros2 launch franka_gazebo_bringup gazebo_joint_position_controller_example.launch.py
```

#### 真实机器人操作
```bash
# 首先在 franka.config.yaml 中配置机器人 IP
ros2 launch franka_bringup franka.launch.py
```

## 配置

### 机器人配置 (`franka_bringup/config/franka.config.yaml`)
```yaml
franka:
  ros__parameters:
    robot_ip: "10.0.0.2"  # 机器人 IP 地址
    namespace: ""         # 机器人命名空间
    use_fake_hardware: false  # 使用仿真
    control_mode: "position"  # 默认控制模式
```

### 控制器配置 (`franka_bringup/config/controllers.yaml`)
- 默认与命名空间无关 (使用 `/**` 通配符)
- 支持每个命名空间的参数覆盖
- 配置控制器类型和参数

### 命名空间支持
系统通过命名空间支持多机器人：
```yaml
# 全局默认
/**:
  franka_robot_state_broadcaster:
    ros__parameters:
      lock_try_count: 5

# 命名空间特定覆盖
/robot1:
  franka_robot_state_broadcaster:
    ros__parameters:
      lock_try_count: 10
```

## 开发

### 创建自定义控制器
1. 继承 `controller_interface::ControllerInterface`
2. 实现必需的生命周期方法
3. 在 `CMakeLists.txt` 中注册为插件
4. 添加到 `controllers.yaml` 配置

控制器结构示例：
```cpp
class MyCustomController : public controller_interface::ControllerInterface {
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update() override;
};
```

### 硬件接口扩展
`FrankaHardwareInterface` 实现 `hardware_interface::SystemInterface` 并提供：
- 命令接口：力、位置、速度、笛卡尔命令
- 状态接口：关节位置、速度、机器人状态
- 模式切换：动态控制模式转换

### 研究集成
- **RL 策略**：使用 `franka_rl_control` 作为策略执行模板
- **感知**：通过 `franka_cameras` 包集成相机
- **遥操作**：扩展 `franka_joystick` 实现自定义遥操作接口

## 测试

### 单元测试
```bash
# 运行特定包的测试
colcon test --packages-select franka_hardware

# 查看测试结果
colcon test-result --verbose
```

### 集成测试
```bash
# 运行集成测试
colcon test --packages-select integration_launch_testing
```

### 仿真测试
- 使用 `use_fake_hardware:=true` 进行无硬件测试
- Gazebo 提供基于物理的仿真
- 仿真和真实硬件之间的一致接口

## 故障排除

### 常见问题

1. **UDP 超时错误**
   - 确保已安装实时内核
   - 避免使用 Docker Desktop (使用 Docker Engine)
   - 检查与机器人的网络连接

2. **控制器未找到**
   - 验证 `controllers.yaml` 中的控制器名称
   - 检查包是否已构建：`colcon list --packages-select`
   - 确保工作空间环境已设置

3. **命名空间问题**
   - 验证启动文件中的命名空间配置
   - 检查 `controllers.yaml` 中的命名空间特定配置
   - 确保主题名称包含命名空间前缀

4. **构建失败**
   - 运行 `rosdep install` 确保依赖项
   - 检查 `franka.repos` 中的外部依赖项
   - 验证 ROS 2 Humble 是否已安装

### 实时配置
对于真实机器人操作：
```bash
# 安装实时内核
sudo apt-get install linux-image-rt

# 配置 Docker 实时性 (如果使用容器)
--cap-add=SYS_NICE --ulimit rtprio=99
```

## 研究应用

### 强化学习
- `franka_rl_control` 提供策略执行的 ROS 2 接口
- 订阅 `FrankaRobotState`，发布 `rl_policy_action`
- 用于模型加载和推理的 PyTorch 集成

### 计算机视觉
- 通过 `franka_cameras` 实现 RealSense 相机集成
- 使用 `franka_sideview_cameras` 支持双相机
- OpenCV 和 ROS 2 图像传输

### 运动规划
- MoveIt2 集成进行运动规划
- OMPL 规划器配置
- RViz 可视化和规划界面

## 贡献指南

1. 遵循 ROS 2 编码标准
2. 使用 `clang-format` 和 `clang-tidy` 保证代码质量
3. 为新功能添加单元测试
4. 为新控制器更新 `controllers.yaml`
5. 在适当的 README 文件中记录新功能

### 提交签名
所有贡献都需要 DCO 签名：
```
Signed-off-by: 您的姓名 <your.email@example.com>
```

## 许可证

所有软件包均根据 Apache 2.0 许可证授权。详见 `LICENSE` 文件。

## 资源

- [官方文档](https://frankaemika.github.io/docs)
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)
- [ROS 2 控制文档](https://control.ros.org/)
- [问题跟踪器](https://github.com/frankaemika/franka_ros2/issues)

## 支持

- GitHub Issues 用于错误报告和功能请求
- ROS Answers 用于使用问题
- Franka 社区用于机器人特定问题
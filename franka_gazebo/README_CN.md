# Franka Gazebo

**==重要提示：==**

`franka_description` 的最低必需版本为 0.3.0。
您可以从 https://github.com/frankaemika/franka_description 克隆 franka_description 包。

一个将 Franka ROS 2 与 Gazebo 仿真器集成的项目。

## 启动 RVIZ + Gazebo

启动一个生成 RVIZ 和 Gazebo 显示机器人的示例：

```bash
ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py
```

如果您想显示其他机器人，可以定义 arm_id：

```bash
ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py arm_id:=fp3
```

如果您想启动包含 franka_hand 的仿真：

```bash
ros2 launch franka_gazebo_bringup visualize_franka_robot.launch.py load_gripper:=true franka_hand:='franka_hand'
```

## Gazebo 关节速度控制示例

启动前，请确保构建 `franka_example_controllers` 和 `franka_description` 包。
`franka_description` 必须具有最低版本 0.3.0。

```bash
colcon build --packages-select franka_example_controllers
```

现在您可以使用 Gazebo 仿真器启动速度示例。

```bash
ros2 launch franka_gazebo_bringup gazebo_joint_velocity_controller_example.launch.py load_gripper:=true franka_hand:='franka_hand'
```

请注意，夹爪关节在关节速度控制器中存在一个错误。
如果您对控制夹爪感兴趣，请使用关节位置接口。

## Gazebo 关节位置控制示例

要运行关节位置控制示例，您需要具备关节速度控制部分列出的所需软件。

然后您可以使用以下命令运行。

```bash
ros2 launch franka_gazebo_bringup gazebo_joint_position_controller_example.launch.py load_gripper:=true franka_hand:='franka_hand'
```

## Gazebo 关节阻抗控制示例

要运行扭矩示例，您必须编译位于 `franka_gazebo` 下的 `franka_ign_ros2_control` 包。
您可以使用以下命令编译 `franka_ign_ros2_control`。

```bash
colcon build --packages-select franka_ign_ros2_control
```

然后设置您的工作空间环境。

```bash
source install/setup.sh
```

然后您可以运行阻抗控制示例。

```bash
ros2 launch franka_gazebo_bringup gazebo_joint_impedance_controller_example.launch.py load_gripper:=true franka_hand:='franka_hand'
```

## Gazebo 游戏手柄笛卡尔位姿控制示例

此示例允许您使用游戏控制器/摇杆控制机器人的末端执行器位姿。

```bash
ros2 launch franka_gazebo_bringup gazebo_joystick_cartesian_pose_controller_example.launch.py load_gripper:=true franka_hand:='franka_hand'
```

**要求：**
- 连接到系统的兼容游戏控制器/摇杆
- 控制器应在系统中正确配置

**注意：** 游戏手柄控制器将摇杆轴映射到笛卡尔位置/姿态命令。有关具体的轴映射，请参考控制器实现。

## 可用启动文件

| 启动文件 | 用途 | 加载的控制器 |
|-------------|---------|-------------------|
| `visualize_franka_robot.launch.py` | 仅可视化（Gazebo + RViz） | `joint_state_broadcaster` |
| `gazebo_joint_velocity_controller_example.launch.py` | 关节速度控制 | `joint_velocity_example_controller` |
| `gazebo_joint_position_controller_example.launch.py` | 关节位置控制 | `joint_position_example_controller` |
| `gazebo_joint_impedance_controller_example.launch.py` | 关节阻抗（扭矩）控制 | `joint_impedance_example_controller` |
| `gazebo_joystick_cartesian_pose_controller_example.launch.py` | 游戏手柄遥操作 | `joystick_cartesian_pose_controller` |

## 通用启动参数

所有启动文件都支持以下参数：

| 参数 | 默认值 | 说明 |
|----------|---------------|-------------|
| `arm_id` | `fr3` | 机器人型号（fr3, fp3, fer） |
| `load_gripper` | `false` | 启用/禁用夹爪 |
| `franka_hand` | `franka_hand` | 夹爪类型 |
| `namespace` | `""` | 机器人命名空间（用于多机器人设置） |

使用自定义参数的示例：
```bash
ros2 launch franka_gazebo_bringup gazebo_joint_position_controller_example.launch.py \
  arm_id:=fr3 \
  load_gripper:=true \
  franka_hand:=franka_hand \
  namespace:=robot1
```

## ROS 2 启动参数语法详解

在ROS 2启动命令中，参数设置使用 `:=` 语法。以下是详细解释：

### 1. 参数赋值语法 `:=`
- `:=` 是ROS 2启动系统的**参数赋值操作符**
- 它将左边的参数名与右边的值关联起来
- 语法格式：`参数名:=参数值`

### 2. 不同参数类型的语法
```bash
# 布尔值 - 直接使用 true/false，不加引号
load_gripper:=true
use_fake_hardware:=false

# 字符串 - 总是用单引号包裹
arm_id:='fr3'
franka_hand:='franka_hand'
namespace:='robot1'

# 包含空格或特殊字符的字符串
franka_hand:='my franka hand'
franka_hand:='hand_v2.0'

# 数字 - 直接使用数字
update_rate:=1000
timeout:=5.0  # 浮点数
```

### 3. 为什么语法如此重要？
```bash
# 正确 - 布尔值不加引号
load_gripper:=true        # → 布尔值 True

# 错误 - 布尔值加引号会变成字符串
load_gripper:='true'      # → 字符串 'true'（不是布尔值）

# 正确 - 字符串加引号
franka_hand:='franka_hand' # → 字符串 'franka_hand'

# 错误 - 字符串不加引号可能被解释为变量名
franka_hand:=franka_hand   # 可能寻找名为 franka_hand 的变量
```

### 4. 在Launch文件中的处理
启动文件通过 `LaunchConfiguration` 接收这些参数：
```python
from launch.substitutions import LaunchConfiguration

# 定义参数
load_gripper = LaunchConfiguration('load_gripper')  # 接收布尔值
franka_hand = LaunchConfiguration('franka_hand')    # 接收字符串

# 使用参数
DeclareLaunchArgument(
    'load_gripper',
    default_value='false',  # 默认值总是字符串
    description='true/false 用于启用夹爪'
)
```

### 5. 最佳实践
1. **布尔值**：直接使用 `true`/`false`，不加引号
2. **字符串**：总是用单引号包裹，即使看起来像普通单词
3. **数字**：直接使用数字，ROS 2会自动处理类型转换
4. **复杂值**：对于包含特殊字符的值，使用适当的引号
5. **多个参数**：用空格分隔多个参数设置

### 6. 参数传递机制
当您运行 `ros2 launch` 命令时：
1. ROS 2解析命令行参数
2. 将 `:=` 右边的值传递给对应的 `LaunchConfiguration`
3. 启动文件中的节点接收这些参数值
4. 参数值通过ROS参数服务器传递给各个节点

这种语法设计使得ROS 2启动系统具有清晰的参数赋值语法和类型安全的参数传递。

## 配置说明

Gazebo 仿真使用位于以下位置的单独控制器配置文件：
```
franka_gazebo_bringup/config/franka_gazebo_controllers.yaml
```

关键配置详情：
- **控制器类型**：定义了关节位置、关节速度、关节阻抗和游戏手柄笛卡尔位姿控制器
- **Gazebo 标志**：所有控制器都有 `gazebo: true` 参数以启用 Gazebo 特定行为
- **阻抗控制增益**：预配置的刚度（k_gains）和阻尼（d_gains）参数
- **更新频率**：控制器管理器以 1000Hz 运行进行仿真

要修改控制器参数，请编辑 YAML 文件并重新构建工作空间：
```bash
colcon build --packages-select franka_gazebo_bringup
```

## 故障排除

如果您遇到 Gazebo 找不到模型文件的问题，请尝试包含工作空间。例如：

```bash
export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}:/workspaces/src/
```
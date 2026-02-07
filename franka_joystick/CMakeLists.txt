cmake_minimum_required(VERSION 3.5)
project(franka_joystick)

# 开启 C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# 编译 joystick_pose_publisher 节点
add_executable(joystick_pose_publisher src/joystick_pose_publisher.cpp)
ament_target_dependencies(joystick_pose_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
)
target_link_libraries(joystick_pose_publisher
  Eigen3::Eigen
)

# 安装可执行文件
install(TARGETS
  joystick_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# <<< 添加这一段，将 launch 文件夹安装到 share 包目录下 >>>
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

cmake_minimum_required(VERSION 3.8)
project(mpk_ethercat_test)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# 生成 MotorData.msg
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorData.msg"
  DEPENDENCIES std_msgs
)

# MotorDataBroadcaster 控制器库
add_library(motor_data_broadcaster SHARED
  src/motor_data_broadcaster.cpp
)
target_include_directories(motor_data_broadcaster PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(motor_data_broadcaster
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# 让控制器库能够使用本包生成的消息类型支持（包含生成的头文件路径等）
rosidl_target_interfaces(motor_data_broadcaster
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

pluginlib_export_plugin_description_file(controller_interface motor_data_broadcaster_plugin.xml)

# C++ 周期下发器（替代原 Python 版本）
add_executable(cyclic_mode_runner
  src/cyclic_mode_runner.cpp
)
ament_target_dependencies(cyclic_mode_runner
  rclcpp
  std_msgs
)
rosidl_target_interfaces(cyclic_mode_runner
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 安装 launch / urdf / config 目录到 share
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS motor_data_broadcaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS cyclic_mode_runner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES motor_data_broadcaster_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Motor Data GUI 脚本（需先 source 工作空间以使用 mpk_ethercat_test.msg）
install(PROGRAMS scripts/motor_data_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
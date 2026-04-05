cmake_minimum_required(VERSION 3.8)
project(airbot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 声明消息文件，由 rosidl 生成 Python/C++ 接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmCommand.msg"
  "msg/EEPose.msg"
  "msg/GripperState.msg"
  "msg/TactileData.msg"
  "msg/SystemMode.msg"
  "msg/EpisodeControl.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_package()

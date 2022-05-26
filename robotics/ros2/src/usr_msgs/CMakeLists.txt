cmake_minimum_required(VERSION 3.5)
project(usr_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Including messages
set(MSG_FILES
  # CAN
  "msg/rpm_converter/MotorsRPM.msg"
  # Location
  "msg/location/LocationMsg.msg"
  # Devices
  "msg/devices/General.msg"
  # Interfaces
  "msg/interfaces/SpeakerStatus.msg"
)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs diagnostic_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
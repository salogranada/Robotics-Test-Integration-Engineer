cmake_minimum_required(VERSION 3.5)
project(wheel_odometry)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(usr_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(utils REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

set(DEPENDENCIES
  rclcpp
  rclcpp_cascade_lifecycle
  std_srvs
  std_msgs
  nav_msgs
  usr_msgs
  sensor_msgs
  geometry_msgs
  utils
  tf2_ros
)

add_executable(wheel_odometry
  src/wheel_odometry.cpp
)

target_link_libraries(wheel_odometry utils::utils)
ament_target_dependencies(wheel_odometry ${DEPENDENCIES})

install(TARGETS
  wheel_odometry
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()

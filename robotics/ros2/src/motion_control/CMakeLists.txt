cmake_minimum_required(VERSION 3.5)
project(motion_control)

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
find_package(std_msgs REQUIRED)
find_package(usr_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(utils)

include_directories(include)

set(DEPENDENCIES
  rclcpp
  rclcpp_cascade_lifecycle
  std_msgs
  usr_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  utils
  std_srvs
)

# Speed controller
add_executable(speed_controller
  src/pid_controller.cpp
  src/speed_controller.cpp
)

target_link_libraries(speed_controller 
  utils::utils 
  ${PROJECT_SOURCE_DIR}/lib/libsoft_speed.a
)

ament_target_dependencies(speed_controller ${DEPENDENCIES})

# Speed controller
install(TARGETS
  speed_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)

ament_package()
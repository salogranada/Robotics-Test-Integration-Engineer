cmake_minimum_required(VERSION 3.5)
project(lifecycle_manager)

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

include_directories(include)


set(DEPENDENCIES
  rclcpp
  rclcpp_cascade_lifecycle
)

add_executable(lifecycle_manager 
  src/lifecycle_manager.cpp)

ament_target_dependencies(lifecycle_manager ${DEPENDENCIES})

target_include_directories(lifecycle_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  lifecycle_manager
  DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()

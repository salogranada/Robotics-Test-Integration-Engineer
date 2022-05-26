cmake_minimum_required(VERSION 3.5)
project(usr_srvs)

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
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(usr_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Including services
set(SRV_FILES
  # CAN
  # "srv/can/FloatValue.srv"
  )

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES} 
  DEPENDENCIES std_msgs usr_msgs sensor_msgs 
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

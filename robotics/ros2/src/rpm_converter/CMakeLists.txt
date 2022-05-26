cmake_minimum_required(VERSION 3.8)
project(rpm_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

add_executable(rpm_converter_node src/rpm_converter_node.cpp)
target_include_directories(rpm_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rpm_converter_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS rpm_converter_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()

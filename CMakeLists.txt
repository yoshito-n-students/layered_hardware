cmake_minimum_required(VERSION 3.8)
project(layered_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_limits REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(transmission_interface REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

## Declare a C++ library
add_library(
  layered_hardware_plugins SHARED
  src/layered_hardware_plugins.cpp)
target_include_directories(
  layered_hardware_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  layered_hardware_plugins
  controller_interface
  controller_manager
  hardware_interface
  joint_limits
  pluginlib
  rclcpp
  rclcpp_components
  transmission_interface
  urdf
  yaml_cpp_vendor)

## Declare a C++ executable

pluginlib_export_plugin_description_file(hardware_interface plugins.xml)
pluginlib_export_plugin_description_file(layered_hardware plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS layered_hardware_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(
  DIRECTORY config launch urdf
  DESTINATION share/layered_hardware)

# TODO: declare tests here

ament_export_include_directories(
  include)

ament_package()

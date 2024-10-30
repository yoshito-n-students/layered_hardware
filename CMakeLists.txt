cmake_minimum_required(VERSION 3.8)
project(layered_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include_directories(include)

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_limits REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(transmission_interface REQUIRED)
# yaml_cpp_vendor builds yaml-cpp if not installed and runs find(yaml-cpp). 
# Explicit linking to ${YAML_CPP_LIBRARIES} is recommended.
find_package(yaml_cpp_vendor REQUIRED)

## Declare plugins as C++ libraries
add_library(
  layered_hardware_hardware_plugins SHARED
  src/layered_hardware_hardware_plugins.cpp)
target_include_directories(
  layered_hardware_hardware_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  layered_hardware_hardware_plugins
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  yaml_cpp_vendor)
target_link_libraries(
  layered_hardware_hardware_plugins
  ${YAML_CPP_LIBRARIES})

add_library(
  layered_hardware_layer_plugins SHARED
  src/layered_hardware_layer_plugins.cpp)
target_include_directories(
  layered_hardware_layer_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  layered_hardware_layer_plugins
  controller_interface
  hardware_interface
  joint_limits
  pluginlib
  rclcpp
  transmission_interface
  yaml_cpp_vendor)
target_link_libraries(
  layered_hardware_layer_plugins
  ${YAML_CPP_LIBRARIES})

# Export plugin descriptions
pluginlib_export_plugin_description_file(hardware_interface hardware_plugins.xml)
pluginlib_export_plugin_description_file(layered_hardware layer_plugins.xml)

# Install resources
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS layered_hardware_hardware_plugins layered_hardware_layer_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(
  DIRECTORY examples
  DESTINATION share/layered_hardware)

# TODO: declare tests here

ament_export_include_directories(
  include)

ament_package()

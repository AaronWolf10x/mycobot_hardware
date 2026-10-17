cmake_minimum_required(VERSION 3.16)
project(mycobot_hardware LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(mycobot_hardware SHARED src/mycobot_system.cpp)
target_compile_features(mycobot_hardware PUBLIC cxx_std_17)
target_link_libraries(mycobot_hardware PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface mycobot_hardware.xml)

install(TARGETS mycobot_hardware DESTINATION lib)
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

ament_package()

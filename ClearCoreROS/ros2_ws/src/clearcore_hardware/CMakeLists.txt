cmake_minimum_required(VERSION 3.16)
project(clearcore_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(clearcore_system SHARED src/clearcore_system.cpp)
target_compile_features(clearcore_system PUBLIC cxx_std_17)
target_include_directories(clearcore_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(clearcore_system
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle)

pluginlib_export_plugin_description_file(hardware_interface clearcore_hardware.xml)

install(TARGETS clearcore_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(clearcore_system)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()

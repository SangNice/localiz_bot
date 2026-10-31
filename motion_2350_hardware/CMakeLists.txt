cmake_minimum_required(VERSION 3.8)
project(motion_2350_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_library(motion_2350_hardware SHARED
  src/motion_2350_system.cpp
)
target_include_directories(motion_2350_hardware PRIVATE include)
ament_target_dependencies(motion_2350_hardware
  rclcpp
  hardware_interface
  pluginlib
  sensor_msgs
)

pluginlib_export_plugin_description_file(hardware_interface motion_2350_hardware.xml)
install(
  FILES motion_2350_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS motion_2350_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
ament_package()

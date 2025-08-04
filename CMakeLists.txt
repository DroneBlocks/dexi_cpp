cmake_minimum_required(VERSION 3.8)
project(dexi_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(dexi_interfaces REQUIRED)
find_package(std_msgs REQUIRED)



include_directories(include)

# PX4 Offboard Manager
add_executable(px4_offboard_manager src/px4_offboard_manager.cpp)
target_include_directories(px4_offboard_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(px4_offboard_manager PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  px4_offboard_manager
  "rclcpp"
  "px4_msgs"
  "dexi_interfaces"
)



# Servo Controller
add_executable(servo_controller src/servo_controller.cpp)
target_include_directories(servo_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(servo_controller PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  servo_controller
  "rclcpp"
  "dexi_interfaces"
)
target_link_libraries(servo_controller stdc++fs)

# RGB Status LED Controller
add_executable(rgb_status_led_controller src/rgb_status_led_controller.cpp)
target_include_directories(rgb_status_led_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(rgb_status_led_controller PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  rgb_status_led_controller
  "rclcpp"
  "dexi_interfaces"
)
target_link_libraries(rgb_status_led_controller stdc++fs)

# TCA9555 Controller
add_executable(tca9555_controller src/tca9555_controller.cpp)
target_include_directories(tca9555_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tca9555_controller PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  tca9555_controller
  "rclcpp"
  "dexi_interfaces"
  "std_msgs"
)

install(TARGETS px4_offboard_manager servo_controller rgb_status_led_controller tca9555_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()

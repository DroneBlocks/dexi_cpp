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

include_directories(include)

add_executable(px4_offboard_manager src/px4_offboard_manager.cpp)
target_include_directories(px4_offboard_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(px4_offboard_manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  px4_offboard_manager
  "rclcpp"
  "px4_msgs"
  "dexi_interfaces"
)

install(TARGETS px4_offboard_manager
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

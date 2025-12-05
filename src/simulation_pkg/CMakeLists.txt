cmake_minimum_required(VERSION 3.8)
project(simulation_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

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

add_executable(fake_uc_bridge src/fake_uc_bridge.cpp)
ament_target_dependencies(fake_uc_bridge rclcpp sensor_msgs std_msgs)

add_executable(fake_vision_node src/fake_vision_node.cpp)
ament_target_dependencies(fake_vision_node rclcpp geometry_msgs)

install(TARGETS
  fake_uc_bridge
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  fake_vision_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

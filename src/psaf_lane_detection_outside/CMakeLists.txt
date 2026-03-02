cmake_minimum_required(VERSION 3.8)
project(psaf_lane_detection_outside)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(psaf_configuration REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rusty_racer_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)


#.......
# according to dependencies


include_directories(include)

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

add_executable(
    lane_detection_outside
    src/lane_detection_outside.cpp
    src/lane_detection_outside_node.cpp
)
ament_target_dependencies(
    lane_detection_outside
    rclcpp psaf_configuration
    sensor_msgs
    OpenCV
    image_transport
    cv_bridge
    rusty_racer_interfaces
    std_msgs
)   #.......  according to dependencies
install(TARGETS
        lane_detection_outside

        DESTINATION lib/${PROJECT_NAME}
)

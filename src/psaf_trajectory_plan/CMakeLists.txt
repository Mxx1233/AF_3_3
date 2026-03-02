cmake_minimum_required(VERSION 3.8)
project(psaf_trajectory_plan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------
# 1. Find Dependencies
# --------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(psaf_configuration REQUIRED)
find_package(rusty_racer_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)

# [NEW] Added these for Image processing
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include)

# --------------------------------------------------------
# 2. Build Executable
# --------------------------------------------------------
add_executable(trajectory_plan src/trajectory_plan.cpp src/trajectory_plan_node.cpp)

ament_target_dependencies(trajectory_plan
  rclcpp
  psaf_configuration
  rusty_racer_interfaces
  OpenCV
  std_msgs
  sensor_msgs
  cv_bridge
)

# --------------------------------------------------------
# 3. Install Rules
# --------------------------------------------------------
install(TARGETS
        trajectory_plan
        DESTINATION lib/${PROJECT_NAME}
)

# --------------------------------------------------------
# 4. Testing
# --------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(trajectory_plan_test test/trajectory_plan_test.cpp src/trajectory_plan_node.cpp)

  ament_target_dependencies(trajectory_plan_test
    rclcpp
    psaf_configuration
    OpenCV
    rusty_racer_interfaces
    std_msgs
    sensor_msgs
    cv_bridge
  )
endif()

# --------------------------------------------------------
# 5. Finalize Package (MUST BE LAST)
# --------------------------------------------------------
ament_package()

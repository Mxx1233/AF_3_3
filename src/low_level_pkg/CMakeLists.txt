cmake_minimum_required(VERSION 3.8)
project(low_level_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rusty_racer_interfaces REQUIRED)

add_library(state_estimation_core INTERFACE)
target_include_directories(state_estimation_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(uc_mapper INTERFACE)
target_include_directories(uc_mapper INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_state_estimation test/test_state_estimation.cpp)
  target_link_libraries(test_state_estimation state_estimation_core)
  if(COVERAGE_RUN)
    target_compile_options(test_state_estimation PRIVATE -fprofile-arcs -ftest-coverage)
    target_link_options(test_state_estimation PRIVATE -fprofile-arcs -ftest-coverage)
  endif()

  ament_add_gtest(test_uc_mapper test/test_uc_mapper.cpp)
  target_link_libraries(test_uc_mapper uc_mapper)

  if(COVERAGE_RUN)
    target_compile_options(test_uc_mapper PRIVATE -fprofile-arcs -ftest-coverage)
    target_link_options(test_uc_mapper PRIVATE -fprofile-arcs -ftest-coverage)
  endif()

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

# ----------------------------------------
#Adding the f0_state_estimation_node
# ----------------------------------------

#add_executable(f0_state_estimation_node src/f0_state_estimation_node.cpp)
#ament_target_dependencies(f0_state_estimation_node rclcpp std_msgs nav_msgs sensor_msgs rusty_racer_interfaces)

#install(TARGETS
#  f0_state_estimation_node
#  DESTINATION lib/${PROJECT_NAME}
#)


# ----------------------------------------
#Adding the a1_state_estimation_node
# ----------------------------------------

add_executable(state_estimation_node src/state_estimation_node.cpp)
ament_target_dependencies(state_estimation_node  rclcpp std_msgs sensor_msgs nav_msgs rusty_racer_interfaces)
target_link_libraries(state_estimation_node state_estimation_core)

install(TARGETS
  state_estimation_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

# ----------------------------------------
# Adding uc_bridge_adapter_node
# ----------------------------------------

add_executable(uc_bridge_adapter_node src/uc_bridge_adapter_node.cpp)
ament_target_dependencies(uc_bridge_adapter_node rclcpp std_msgs rusty_racer_interfaces)
target_link_libraries(uc_bridge_adapter_node uc_mapper)

install(TARGETS
  uc_bridge_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)
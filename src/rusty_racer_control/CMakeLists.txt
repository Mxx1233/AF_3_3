cmake_minimum_required(VERSION 3.8)
project(rusty_racer_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================================
# Abhängigkeiten / Dependencies
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rusty_racer_interfaces REQUIRED)

# ============================================================================
# Include-Verzeichnisse / Include Directories
# ============================================================================
include_directories(include)

# ============================================================================
# Ausführbare Datei / Executable
# ============================================================================
add_executable(control_node src/control_node.cpp)

ament_target_dependencies(control_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  rusty_racer_interfaces
)

# Jazzy: explicitly link rosidl-generated message targets
target_link_libraries(control_node
  ${rusty_racer_interfaces_TARGETS}
)

# ============================================================================
# Installation
# ============================================================================
install(TARGETS
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
<<<<<<< HEAD
        DESTINATION include
=======
  DESTINATION include
>>>>>>> eabb7b2 (feat: integrate TrafficFSM, simplified controllers, updated interfaces)
)

# ============================================================================
# Test-Konfiguration / Testing Configuration
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_lint_common REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_traffic_fsm
    test/test_traffic_fsm.cpp
  )
  target_include_directories(test_traffic_fsm PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_options(test_traffic_fsm PRIVATE -Wall -Wextra)
  set_tests_properties(test_traffic_fsm PROPERTIES TIMEOUT 60)

  ament_add_gtest(test_controllers
    test/test_controllers.cpp
  )
  target_include_directories(test_controllers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_options(test_controllers PRIVATE -Wall -Wextra)
  set_tests_properties(test_controllers PROPERTIES TIMEOUT 60)

  ament_add_gtest(test_control_integration
    test/test_control_integration.cpp
  )
  target_include_directories(test_control_integration PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_control_integration
    rclcpp
    rusty_racer_interfaces
  )
  target_link_libraries(test_control_integration
    ${rusty_racer_interfaces_TARGETS}
  )
  target_compile_options(test_control_integration PRIVATE -Wall -Wextra)
  set_tests_properties(test_control_integration PROPERTIES TIMEOUT 60)
endif()

# ============================================================================
# Paket exportieren / Export package
# ============================================================================
ament_package()
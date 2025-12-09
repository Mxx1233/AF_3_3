cmake_minimum_required(VERSION 3.8)
project(rusty_racer_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Abhängigkeiten / Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rusty_racer_interfaces REQUIRED)

# Include-Verzeichnisse / Include Directories
include_directories(include)

# Ausführbare Datei / Executable
add_executable(control_node src/control_node.cpp)
ament_target_dependencies(control_node
        rclcpp
        std_msgs
        geometry_msgs
        nav_msgs
        tf2
        tf2_geometry_msgs
        rusty_racer_interfaces
)

# Installation
install(TARGETS
        control_node
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

# ============================================================================
# Test-Konfiguration / Testing Configuration
# ============================================================================
if(BUILD_TESTING)
  # Test-Abhängigkeiten finden / Find test dependencies
  find_package(ament_cmake_gtest REQUIRED)

  # GTest Unit-Tests hinzufügen / Add GTest unit tests
  ament_add_gtest(${PROJECT_NAME}_test
          test/test_control_node.cpp
  )

  # Test-Ziel-Abhängigkeiten verknüpfen / Link test target dependencies
  target_include_directories(${PROJECT_NAME}_test PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_NAME}_test
          rclcpp
          geometry_msgs
          nav_msgs
  )

endif()

ament_package()

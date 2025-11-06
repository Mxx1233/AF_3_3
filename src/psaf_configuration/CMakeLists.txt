cmake_minimum_required(VERSION 3.5)
project(psaf_configuration)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)


add_library(psaf_configuration SHARED
        src/configuration.cpp
        )

ament_target_dependencies(psaf_configuration rclcpp)

target_include_directories(
        psaf_configuration PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_export_targets(psaf_configuration HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)

install(
        DIRECTORY include/
        DESTINATION include
)

install(
        TARGETS psaf_configuration
        EXPORT psaf_configuration
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

ament_export_dependencies(rclcpp)
ament_export_targets(psaf_configuration HAS_LIBRARY_TARGET)
ament_export_libraries(psaf_configuration)
ament_export_include_directories(include)

ament_package()

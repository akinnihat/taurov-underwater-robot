cmake_minimum_required(VERSION 3.5)
project(mec319_pkg_cpp)

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
find_package(OpenCV REQUIRED)
find_package(mec319_interfaces REQUIRED)

add_executable(object_status_publisher src/object_status_publisher.cpp)
ament_target_dependencies(object_status_publisher rclcpp mec319_interfaces OpenCV)

install(TARGETS
  object_status_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

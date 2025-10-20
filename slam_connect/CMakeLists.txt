cmake_minimum_required(VERSION 3.8)
project(slam_connect)

# Default compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Required for all ROS 2 packages
find_package(ament_cmake REQUIRED)

# Install config files, launch files, etc.
install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

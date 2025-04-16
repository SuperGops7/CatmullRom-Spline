cmake_minimum_required(VERSION 3.8)
project(spline_creator)

# Use C++14 (or another standard if you prefer).
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the required ROS 2 packages.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# Add this package's include folder to the include path.
# 'include' is the folder containing 'spline_creator/catmull_rom.hpp' etc.
include_directories(
  include
)

# Build an executable named 'spline_creator_node'
# from your three source files in the 'src' directory.
add_executable(spline_creator_node
  src/catmull_rom.cpp
  src/path_publisher.cpp
  src/main.cpp
)

# Link the ROS 2 libraries and declare their dependencies.
ament_target_dependencies(spline_creator_node
  rclcpp
  geometry_msgs
  nav_msgs
)

# Install the executable so it becomes available as a ROS 2 node.
install(TARGETS
  spline_creator_node
  DESTINATION lib/${PROJECT_NAME}
)

# If you have a 'launch' folder or other resources, you can install them as well:
install(DIRECTORY launch/ rviz DESTINATION share/${PROJECT_NAME}
)

# Finalize the ament package.
ament_package()

cmake_minimum_required(VERSION 3.5)
project(cohan_nav2_tutorial)

# Default to C++14/20 if needed (optional for Python-only packages)
# set(CMAKE_CXX_STANDARD 14)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install maps directory
install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install config directory
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz directory
install(
  DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts (Python executables)
install(
  PROGRAMS
    scripts/agents_bridge.py
    scripts/activate_lifecylenodes_with_delay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_package()

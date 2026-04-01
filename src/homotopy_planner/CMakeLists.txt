cmake_minimum_required(VERSION 3.5)
project(homotopy_planner)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Set to Release in order to speed up the program significantly
set(CMAKE_BUILD_TYPE Release) # None, Debug, Release, RelWithDebInfo, MinSizeRel

# Find ament packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(hateb_local_planner REQUIRED)

# Include directories
include_directories(
  include
)

# Set dependencies for ament_target_dependencies
set(dependencies
  ament_index_cpp
  rclcpp
  nav2_core
  nav_msgs
  std_msgs
  pluginlib
  nav2_navfn_planner
  geometry_msgs
  hateb_local_planner
)

add_library(homotopy_planner SHARED
  src/homotopy_planner.cpp)

install(TARGETS homotopy_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(FILES homotopy_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME})

ament_target_dependencies(homotopy_planner
  rclcpp nav2_core nav2_navfn_planner nav_msgs pluginlib hateb_local_planner)

pluginlib_export_plugin_description_file(nav2_core homotopy_planner_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(homotopy_planner)
ament_export_dependencies(${dependencies})

ament_package()

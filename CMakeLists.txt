cmake_minimum_required(VERSION 3.8)
project(nav_robot_cpp)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)
set(DEPS rclcpp rclcpp_lifecycle pluginlib nav2_core nav2_util nav2_costmap_2d geometry_msgs nav_msgs
         sensor_msgs tf2 tf2_ros tf2_geometry_msgs)

# standalone node
add_executable(scan_watchdog src/scan_watchdog.cpp)
ament_target_dependencies(scan_watchdog rclcpp sensor_msgs)

# Nav2 plugins
add_library(nav_robot_plugins SHARED
  src/straight_line_planner.cpp
  src/pure_pursuit_controller.cpp
  src/log_waypoint_task.cpp)
ament_target_dependencies(nav_robot_plugins ${DEPS})
pluginlib_export_plugin_description_file(nav2_core nav_robot_plugins.xml)
# Humble's waypoint follower loads task executors from its own package's export list
pluginlib_export_plugin_description_file(nav2_waypoint_follower nav_robot_waypoint_plugins.xml)

# behavior tree plugin (loaded by name from bt_navigator's plugin_lib_names)
add_library(nav_robot_scan_healthy_bt_node SHARED src/scan_healthy_condition.cpp)
ament_target_dependencies(nav_robot_scan_healthy_bt_node rclcpp sensor_msgs behaviortree_cpp_v3)
target_compile_definitions(nav_robot_scan_healthy_bt_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS scan_watchdog DESTINATION lib/${PROJECT_NAME})
install(TARGETS nav_robot_plugins nav_robot_scan_healthy_bt_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES nav_robot_plugins.xml nav_robot_waypoint_plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_logic test/test_logic.cpp)
  target_include_directories(test_logic PRIVATE include)
endif()

ament_export_libraries(nav_robot_plugins)
ament_export_dependencies(${DEPS})
ament_package()

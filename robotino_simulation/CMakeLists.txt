cmake_minimum_required(VERSION 3.5)
project(robotino_simulation)

# Find packages
find_package(ament_cmake_auto REQUIRED)
find_package(rclpy REQUIRED)
find_package(mps_description REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(webots_ros2_driver REQUIRED)

pluginlib_export_plugin_description_file(webots_ros2_driver robotino_driver.xml)
# MyRobotDriver library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/robotino_driver.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  rclcpp
  webots_ros2_driver
  nav_msgs
  tf2_ros
  sensor_msgs
)

ament_auto_find_build_dependencies()
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch rviz protos worlds config
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  PROGRAMS
  robotino_simulation/mps_publisher.py
  robotino_simulation/robotinobase1_driver_plugin.py
  robotino_simulation/robotinobase2_driver_plugin.py
  robotino_simulation/robotinobase3_driver_plugin.py
  robotino_simulation/robotino_driver_plugin.py
  DESTINATION lib/${PROJECT_NAME}
)

file(MAKE_DIRECTORY "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")
execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink
	${mps_description_DIR}/../meshes
    ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/mps_meshes
)
ament_export_dependencies(rclpy)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()

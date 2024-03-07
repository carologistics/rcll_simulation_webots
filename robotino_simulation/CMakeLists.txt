cmake_minimum_required(VERSION 3.5)
project(robotino_simulation)

# Find packages
find_package(ament_cmake_auto REQUIRED)
find_package(rclpy REQUIRED)
find_package(mps_description REQUIRED)
ament_auto_find_build_dependencies()
ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch rviz protos worlds config
  DESTINATION share/${PROJECT_NAME}
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

ament_package()

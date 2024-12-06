cmake_minimum_required(VERSION 3.8)
project(rviz_plugins)

# Enable additional warnings for GCC/Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robotino_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(Qt5Widgets REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

set(library_name ${PROJECT_NAME})

# Add library
add_library(${library_name} SHARED
  src/pose_service_panel.cpp
)

# Include directories
target_include_directories(${library_name} PUBLIC
  ${Qt5Widgets_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Link Qt5 libraries
target_link_libraries(${library_name}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  rviz_common::rviz_common
  pluginlib::pluginlib
  rviz_rendering::rviz_rendering
)

# Install library
install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install include files
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export plugin description
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Specify ament dependencies
ament_target_dependencies(${library_name}
  geometry_msgs
  nav2_msgs
  nav2_util
  rclcpp
  rviz_common
  rviz_default_plugins
  std_msgs
  robotino_interface
  pluginlib
  rviz_ogre_vendor
  rviz_rendering
  visualization_msgs
  Qt5
  Qt5Widgets
)

ament_package()

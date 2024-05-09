cmake_minimum_required(VERSION 3.8)
project(hri_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

find_package(rclcpp REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rcpputils REQUIRED)

find_package(pluginlib REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)

find_package(hri REQUIRED)
find_package(hri_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

## This setting causes Qt's "MOC" generation to happen automatically, which is required for
## Qt's signal/slot connections to work.
set(CMAKE_AUTOMOC ON)

set(HEADER_FILES
  include/hri_rviz/hri_humans.hpp
  include/hri_rviz/hri_skeletons.hpp
  include/hri_rviz/frame_info.hpp
  include/hri_rviz/hri_tf.hpp
)

foreach(header "${HEADER_FILES}")
  qt5_wrap_cpp(MOCED_HEADER_FILES "${header}")
endforeach()

set(SRC_FILES
  src/hri_humans.cpp
  src/hri_skeletons.cpp
  src/frame_info.cpp
  src/frame_selection_handler.cpp
  src/hri_tf.cpp
)

add_library(${PROJECT_NAME} SHARED
  ${MOCED_HEADER_FILES}
  ${SRC_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  rviz_common::rviz_common
)

ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
  pluginlib
  rcpputils
  rclcpp
  hri
  hri_msgs
  sensor_msgs
  rviz_common
  rviz_ogre_vendor
  rviz_default_plugins
  cv_bridge
)

# Export the plugin to be imported by rviz2
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rcpputils
  rclcpp
  hri
  hri_msgs
  rviz_common
  rviz_default_plugins
  cv_bridge
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()

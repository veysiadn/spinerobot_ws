cmake_minimum_required(VERSION 3.5)

project(cam_pkg)

find_package(ament_cmake REQUIRED)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
## This is for joystick/Controller_node
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ecat_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(cam_pkg_SRCS
  src/cam_pkg/image_view.cpp
)

set(cam_pkg_HDRS
  include/cam_pkg/image_view.h
)

set(cam_pkg_UIS
  src/cam_pkg/image_view.ui
)

set(cam_pkg_INCLUDE_DIRECTORIES
  include
  ${rclcpp_INCLUDE_DIRS}
  ${qt_gui_cpp_INCLUDE_DIRS}
  ${rqt_gui_cpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

qt5_wrap_cpp(cam_pkg_MOCS ${cam_pkg_HDRS})

qt5_wrap_ui(cam_pkg_UIS_H ${cam_pkg_UIS})

include_directories(
  ${cam_pkg_INCLUDE_DIRECTORIES}
)

add_library(${PROJECT_NAME} SHARED
  ${cam_pkg_SRCS}
  ${cam_pkg_MOCS}
  ${cam_pkg_UIS_H}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  ecat_msgs
  std_msgs
  rqt_gui_cpp
  cv_bridge
  )


target_link_libraries(
  ${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${qt_gui_cpp_LIBRARIES}
  ${rqt_gui_cpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  Qt5::Widgets
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME}
  INCLUDES DESTINATION include)

install(PROGRAMS scripts/cam_pkg
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()

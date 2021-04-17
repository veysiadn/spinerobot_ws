cmake_minimum_required(VERSION 3.5)
project(ecat_pkg)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -I/opt/etherlab/include -L/opt/etherlab/lib -lethercat -Wl,--rpath -Wl,/opt/etherlab/lib)


set(etherlab_include /opt/etherlab/include)
set(etherlab_lib /opt/etherlab/lib/libethercat.so.1.1.0)
set(ecat_node_include ~/spinerobot_ws/src/ecat_pkg/include/ecat_pkg/)
set(node_name "ecat_node")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_executable(ecat_node src/main.cpp
			 src/ecat_node.cpp
			 src/ecat_slave.cpp)

target_include_directories(ecat_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${etherlab_include})

target_link_libraries(ecat_node
${etherlab_lib}
)
# Add include directories
include_directories(
  ${etherlab_include}
  ${ecat_node_include}
)
ament_target_dependencies(${node_name}  rclcpp rclcpp_lifecycle)

install(TARGETS ecat_node
  DESTINATION lib/${PROJECT_NAME})


ament_package()

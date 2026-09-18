cmake_minimum_required(VERSION 3.8)
project(chapt10_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)
find_path(CPPAD_INCLUDE_DIR cppad/ipopt/solve.hpp REQUIRED)
find_library(IPOPT_LIBRARY NAMES ipopt HINTS ${IPOPT_LIBRARY_DIRS} REQUIRED)

add_executable(mpc_node
  src/mpc_node.cpp
  src/mpc.cpp)

target_include_directories(mpc_node PRIVATE
  include
  ${CPPAD_INCLUDE_DIR}
  ${IPOPT_INCLUDE_DIRS})
target_compile_options(mpc_node PRIVATE ${IPOPT_CFLAGS_OTHER})
target_link_directories(mpc_node PRIVATE ${IPOPT_LIBRARY_DIRS})
target_link_options(mpc_node PRIVATE -Wl,--allow-shlib-undefined)
target_link_libraries(mpc_node Eigen3::Eigen ${IPOPT_LIBRARY})

ament_target_dependencies(mpc_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS mpc_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

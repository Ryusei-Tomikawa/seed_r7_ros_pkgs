cmake_minimum_required(VERSION 2.8.3)
project(seed_r7_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
)

catkin_package()

## Gazebo plugins
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories( ${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(mimic_plugin src/MimicPlugin.cpp)
install(TARGETS mimic_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_library(gazebo_ros_planar_force_move src/gazebo_ros_planar_move.cpp)
install(TARGETS gazebo_ros_planar_force_move LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

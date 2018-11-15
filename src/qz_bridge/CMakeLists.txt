cmake_minimum_required(VERSION 2.8.3)
project(qz_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qz_bridge
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

# set our dependent libs for qingzhui robot
set(GENERAL_INCL_DIR "/usr/Aris_Dependent")
set(SYSTEM_LINK_LIB pthread zmq)
set(XENOMAI_INCL_DIR "/usr/Aris_Dependent/xenomai/include")
set(XENOMAI_LINK_DIR "/usr/Aris_Dependent/xenomai/lib")
set(RPDJSON_INCL_DIR "/usr/Aris_Dependent/rapidjson/include")
set(ARIS_INCL_DIR "/usr/aris/include")
set(ARIS_LINK_DIR "/usr/aris/lib")
set(NRT_ARIS_LINK_LIB  aris_core)
set(EIGEN_INCL_DIR "/usr/Aris_Dependent/Eigen/")
set(ROBOT_LIBS_INCL_DIR "/usr/robot_libs/include/Kinematics" "/usr/robot_libs/include/Algorithms/" "/usr/robot_libs/include/DataNetworking/")
set(ROBOT_LIBS_LINK_DIR "/usr/robot_libs/lib")
set(ROBOT_LIBS_LINK_LIB  DataNetworking Kinematics RbtAlgorithm)

set(ALL_INCL_DIR ${ARIS_INCL_DIR} ${XENOMAI_INCL_DIR} ${GENERAL_INCL_DIR} ${RPDJSON_INCL_DIR} ${EIGEN_INCL_DIR} ${ROBOT_LIBS_INCL_DIR})
set(NRT_LINK_DIR ${ARIS_LINK_DIR} ${ROBOT_LIBS_LINK_DIR} )
set(NRT_LINK_LIB ${NRT_ARIS_LINK_LIB} ${SYSTEM_LINK_LIB} ${ROBOT_LIBS_LINK_LIB})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${ALL_INCL_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${NRT_LINK_DIR})

add_executable(cmd_vel_bridge src/cmd_vel_bridge.cpp)
target_link_libraries(cmd_vel_bridge ${catkin_LIBRARIES} ${NRT_LINK_LIB})

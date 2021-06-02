cmake_minimum_required(VERSION 2.8.3)
project(qz_bridge)
set( CMAKE_EXPORT_COMPILE_COMMANDS 1)
set( CMAKE_BUILD_TYPE RELEASE)
set( BRIDGE_PROTOCOL_VERSION 1)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  std_msgs
  std_srvs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  GaitPhase.msg
  RobotTipState.msg
  GaitPhaseV2.msg
  RobotTipStateV2.msg
) 

generate_messages(
  DEPENDENCIES
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


###########
## Build ##
###########

# set our dependent libs for qingzhui robot
set(GENERAL_INCL_DIR "/usr/Aris_Dependent")
set(SYSTEM_LINK_LIB pthread zmq)
set(XENOMAI_INCL_DIR "/usr/xenomai/include")
set(RPDJSON_INCL_DIR "/usr/local/include/rapidjson")
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

if (BRIDGE_PROTOCOL_VERSION EQUAL 1)
  add_definitions(-DBRIDGE_PROTOCOL_VERSION_1)
elseif (BRIDGE_PROTOCOL_VERSION EQUAL 2)
  add_definitions(-DBRIDGE_PROTOCOL_VERSION_2)
endif()

add_executable(cmd_vel_bridge src/cmd_vel_bridge.cpp)
target_link_libraries(cmd_vel_bridge ${catkin_LIBRARIES} ${NRT_LINK_LIB})

add_executable(qz_state_publisher src/qz_state_publisher.cpp src/QzRobotStatePublisher.cpp)
target_link_libraries(qz_state_publisher ${catkin_LIBRARIES} ${NRT_LINK_LIB})

cmake_minimum_required(VERSION 3.0.2)
project(uav_config)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
# find_package(lcm REQUIRED)
# we have to use lcm-gen -x PoseTwistAccStamped.lcm to generate the header file

catkin_package(
  INCLUDE_DIRS include types
#  LIBRARIES libuav_config
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  types
)


add_executable(parse_demo src/uav_config_parse.cpp)
target_link_libraries(parse_demo ${catkin_LIBRARIES} yaml-cpp)
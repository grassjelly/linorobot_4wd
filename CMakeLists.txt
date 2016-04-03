cmake_minimum_required(VERSION 2.8.3)
project(linorobot4wd)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ros_arduino_msgs
  roscpp
  rospy
  rosserial_python
  sensor_msgs
  std_msgs
  tf
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES linorobot
#  CATKIN_DEPENDS geometry_msgs ros_arduino_msgs roscpp rospy rosserial_python sensor_msgs std_msgs tf tf
#  DEPENDS system_lib
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lino_base_node_4wd src/lino_base_node_4wd.cpp)
target_link_libraries(lino_base_node_4wd ${catkin_LIBRARIES})

add_executable(base_laser_tf_4wd src/base_laser_tf_4wd.cpp)
target_link_libraries(base_laser_tf_4wd ${catkin_LIBRARIES})

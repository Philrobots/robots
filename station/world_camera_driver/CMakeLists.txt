cmake_minimum_required(VERSION 2.8.3)
project(world_camera_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  usb_cam
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES world_camera_driver
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

cmake_minimum_required(VERSION 3.0.2)
project(moveit_joystick_control)

## Add support for Python
find_package(catkin REQUIRED COMPONENTS
  rospy
  moveit_commander
  geometry_msgs
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/test.py  # Make sure to use the correct script name and location
  src/test_joy.py  # Make sure to use the correct script name and location
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

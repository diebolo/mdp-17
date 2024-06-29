cmake_minimum_required(VERSION 3.0.2)
project(mirte-clean)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  actionlib
  frontier_exploration
  smach
  smach_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  src/mirte_clean/state_control.py
  src/mirte_clean/shutdown_service.py
  src/mirte_clean/robot_state_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


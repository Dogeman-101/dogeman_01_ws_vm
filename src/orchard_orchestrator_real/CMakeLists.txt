cmake_minimum_required(VERSION 3.0.2)
project(orchard_orchestrator_real)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  move_base_msgs
  actionlib
  tf
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/phase1/single_goal_debug.py
  scripts/phase1/waypoint_runner.py
  scripts/algorithms/hungarian.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

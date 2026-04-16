cmake_minimum_required(VERSION 3.0.2)
project(orchard_task_assignment)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  scripts/task_assigner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

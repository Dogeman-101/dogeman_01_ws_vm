cmake_minimum_required(VERSION 3.0.2)
project(orchard_orchestrator)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
  scripts/orchestrator.py
  scripts/hungarian_utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

cmake_minimum_required(VERSION 3.0.2)
project(mocap_localization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  tf2_geometry_msgs
  tf2_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mocap_to_tf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

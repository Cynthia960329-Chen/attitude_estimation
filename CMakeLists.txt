cmake_minimum_required(VERSION 3.0.2)
project(attitude_estimation)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/ekf_attitude_estimation.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/rivz_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


cmake_minimum_required(VERSION 3.0.2)
project(viz_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/plane_visualizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

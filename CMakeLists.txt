cmake_minimum_required(VERSION 3.0.2)
project(viz_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  visualization_msgs
  moveit_msgs
  shape_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs visualization_msgs moveit_msgs shape_msgs
)

catkin_install_python(PROGRAMS
  scripts/viz_obj.py
  scripts/clear_env.py
  scripts/list_models.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

# Ensure STL and assets are available after install
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY modol/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/modol
)

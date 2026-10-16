cmake_minimum_required(VERSION 3.0.2)
project(race_abot_411)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  abot_driver
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs nav_msgs cv_bridge abot_driver
)

# 全部为 Python 节点，无需编译 C++
catkin_install_python(PROGRAMS
  scripts/race_abot_411_node.py
  scripts/race_abot_411_sign_detect.py
  scripts/race_abot_411_fake_sign.py
  scripts/race_abot_411_diagnostics.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

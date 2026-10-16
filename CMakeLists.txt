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

# 纯 Python 包：不需要导出任何 catkin/C++ 接口，所以 catkin_package() 留空。
# 注意：不要在 CATKIN_DEPENDS 里列包——那要求 package.xml 里对应写成 <depend>（build 依赖），
#      否则会报 "the catkin package 'rospy' has been find_package()-ed but is not listed
#      as a build dependency in the package.xml"。运行期依赖写在 package.xml 的 <exec_depend> 即可。
catkin_package()

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

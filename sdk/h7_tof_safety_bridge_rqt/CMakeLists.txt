cmake_minimum_required(VERSION 3.0.2)
project(h7_tof_safety_bridge_rqt)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  h7_tof_safety_bridge
  rospy
  rqt_gui
  rqt_gui_py
  std_srvs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure h7_tof_safety_bridge rospy rqt_gui rqt_gui_py std_srvs
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

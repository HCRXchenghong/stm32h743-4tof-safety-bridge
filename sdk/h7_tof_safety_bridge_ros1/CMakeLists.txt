cmake_minimum_required(VERSION 3.0.2)
project(h7_tof_safety_bridge)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  diagnostic_msgs
  message_generation
  rospy
  std_msgs
  std_srvs
)

add_message_files(
  FILES
  TofSafetyStatus.msg
)

generate_dynamic_reconfigure_options(
  cfg/SafetyBridge.cfg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS diagnostic_msgs dynamic_reconfigure message_runtime rospy std_msgs std_srvs
)

catkin_install_python(PROGRAMS
  scripts/h7_tof_bridge_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

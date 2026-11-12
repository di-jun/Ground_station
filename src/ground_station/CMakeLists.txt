cmake_minimum_required(VERSION 3.0.2)
project(ground_station)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs nav_msgs geometry_msgs mavros_msgs)

catkin_package(CATKIN_DEPENDS rospy std_msgs nav_msgs geometry_msgs mavros_msgs)

catkin_install_python(PROGRAMS
  scripts/ground_station_node.py
  scripts/ground_station_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
cmake_minimum_required(VERSION 2.8.3)
project(scxml_interpreter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  smach
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy smach
)
install(FILES
   tests/resources
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_add_nosetests(test)

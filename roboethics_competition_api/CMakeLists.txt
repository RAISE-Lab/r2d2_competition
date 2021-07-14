cmake_minimum_required(VERSION 2.8.3)
project(roboethics_competition_api)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)
catkin_python_setup()
catkin_package()

install(DIRECTORY config launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

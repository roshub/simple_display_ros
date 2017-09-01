cmake_minimum_required(VERSION 2.8.3)
project(simple_display_launch)

find_package(catkin REQUIRED roslaunch)

catkin_package(
  CATKIN_DEPENDS roslaunch
)


roslaunch_add_file_check(launch)

install(FILES
  launch/primary.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

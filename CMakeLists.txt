cmake_minimum_required(VERSION 3.0.2)
project(plexil_actionlib)

find_package(catkin REQUIRED COMPONENTS actionlib)
find_package(PLEXIL REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${PLEXIL_INCLUDE_DIR}
  LIBRARIES ${catkin_LIBRARIES}
  CATKIN_DEPENDS actionlib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

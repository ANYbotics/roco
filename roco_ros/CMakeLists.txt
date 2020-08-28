cmake_minimum_required(VERSION 3.5.1)
project(roco_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CATKIN_PACKAGE_DEPENDENCIES
  message_logger
  roco
  roscpp
  )

find_package(catkin REQUIRED
  COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  )

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  CATKIN_DEPENDS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

#############
## Install ##
#############

install(
  DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#################
## Clang Tools ##
#################

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)

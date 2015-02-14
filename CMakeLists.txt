#-------------------------------------------------------------------------------
# Autonomous Systems Lab
# ETH Zurich
#
# Author(s): Christian Gehring
# email    : gehrinch@ethz.ch
# Date     : 1-Dec-2014
#
#-------------------------------------------------------------------------------
cmake_minimum_required (VERSION 2.8)

project(roco)

# Find Catkin
find_package(catkin REQUIRED)

# Declare this project as a catkin package
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES roco
# CATKIN_DEPENDS 
# DEPENDS system_lib
)

if(DEFINED ENV{ROS_DISTRO})
message(STATUS "Building for ros.")
add_definitions(-DUSE_ROS)
endif()

set(CMAKE_CXX_FLAGS "-fPIC")
add_definitions(--std=c++11)

include_directories(include)

add_subdirectory(src/controllers)
add_subdirectory(src/time)

add_library(roco SHARED ${CTRLS_SRCS})
target_link_libraries(roco ${CTRLS_LIBS})


# Add Doxygen documentation
if (BUILD_DOC)
add_subdirectory(doc/doxygen)
endif (BUILD_DOC)


if (BUILD_EXAMPLE)
add_subdirectory(example example)
endif()

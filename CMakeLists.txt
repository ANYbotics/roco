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

if(DEFINED ENV{ROS_DISTRO})
    message(STATUS "Building for ros.")
    catkin_package(
     INCLUDE_DIRS include
     LIBRARIES roco
     CATKIN_DEPENDS roscpp
     DEPENDS 
     CFG_EXTRAS roco.cmake 
    )
    add_definitions(-DUSE_ROS)
else()
    catkin_package(
     INCLUDE_DIRS include
     LIBRARIES roco
     CATKIN_DEPENDS 
     DEPENDS
     CFG_EXTRAS roco.cmake 
    )
endif()

set(CMAKE_CXX_FLAGS "-fPIC")
add_definitions(--std=c++11)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(src/controllers)
add_subdirectory(src/time)

add_library(roco SHARED ${CTRLS_SRCS})
target_link_libraries(roco ${CTRLS_LIBS})
target_link_libraries(roco ${catkin_LIBRARIES})


# Add Doxygen documentation
if (BUILD_DOC)
add_subdirectory(doc/doxygen)
endif (BUILD_DOC)


if (BUILD_EXAMPLE)
add_subdirectory(example example)
endif()

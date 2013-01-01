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

find_package(Boost REQUIRED COMPONENTS thread)

# Find Catkin
find_package(catkin REQUIRED)

if(DEFINED ENV{ROS_DISTRO})
    message(STATUS "Building for ros.")
    catkin_package(
     INCLUDE_DIRS include
     LIBRARIES roco
     CATKIN_DEPENDS roscpp
     DEPENDS boost
     CFG_EXTRAS roco.cmake 
    )
    add_definitions(-DUSE_ROS)
else()
    catkin_package(
     INCLUDE_DIRS include
     LIBRARIES roco
     CATKIN_DEPENDS 
     DEPENDS boost
     CFG_EXTRAS roco.cmake 
    )
endif()

set(CMAKE_CXX_FLAGS "-fPIC")
add_definitions(--std=c++11)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_library(roco SHARED 
    src/time/Time.cpp  
    src/time/TimeStd.cpp  
    src/workers/Worker.cpp
    src/controllers/ControllerAdapterInterface.cpp
)

target_link_libraries(roco ${catkin_LIBRARIES})
target_link_libraries(roco ${Boost_LIBRARIES})

# Add Doxygen documentation
if (BUILD_DOC)
add_subdirectory(doc/doxygen)
endif (BUILD_DOC)


if (BUILD_EXAMPLE)
add_subdirectory(example example)
endif()

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
find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

if(DEFINED ENV{ROS_DISTRO})
    message(STATUS "Building for ros.")
    catkin_package(
     INCLUDE_DIRS include
     LIBRARIES roco
     CATKIN_DEPENDS roscpp
     DEPENDS Boost
     CFG_EXTRAS roco.cmake 
    )
    add_definitions(-DUSE_ROS)
else()
    catkin_package(
     INCLUDE_DIRS include
     LIBRARIES roco
     CATKIN_DEPENDS 
     DEPENDS Boost
     CFG_EXTRAS roco.cmake 
    )
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
add_definitions(--std=c++11)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

add_library(roco SHARED 
    src/time/Time.cpp  
    src/time/TimeStd.cpp  
    src/workers/Worker.cpp
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


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

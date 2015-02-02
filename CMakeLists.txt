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

set(EXECUTABLE_OUTPUT_PATH 	${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH 	${PROJECT_SOURCE_DIR}/lib)


# Noisily default to Release build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  	message(STATUS "Setting build type to 'Release' as none was specified.")
  	set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
	"Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(DEFINED ENV{ROS_DISTRO})
message(STATUS "Building for ros.")
add_definitions(-DUSE_ROS)
endif()


set(CMAKE_CXX_FLAGS "-fPIC")
add_definitions(--std=c++11)



# Add CMake module path
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
find_package(Kindr)


include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Kindr_INCLUDE_DIRS})
include_directories(include)


add_subdirectory(src/controllers)
add_subdirectory(src/time)

set(CTRLS_SRCS ${CTRLS_SRCS} blabla)

add_library(roco SHARED ${CTRLS_SRCS})
target_link_libraries(roco ${CTRLS_LIBS})


# Add Doxygen documentation
if (BUILD_DOC)
add_subdirectory(doc/doxygen)
endif (BUILD_DOC)



# Generate FindRobotUtils.cmake file
file(WRITE cmake/FindRoco.cmake
"# This file was automatically generated during the installation of the robot utils library
# and can be used through cmake to find the corresponding header files. A copy of this
# file was created in ${CMAKE_ROOT}/Modules (depending on the CMAKE_ROOT variable).

set(ROCO_INCLUDE_DIRS
    ${CMAKE_INSTALL_PREFIX}/include/roco/include
)

set(ROCO_LIBRARIES roco)

set(ROCO_FOUND TRUE)
"
)


# Setting for make install
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/include/roco)")
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX}/include/roco)
install(FILES cmake/FindRoco.cmake DESTINATION ${CMAKE_ROOT}/Modules)
install(FILES lib/libroco.so DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

if (BUILD_EXAMPLE)
add_subdirectory(example example)
endif()

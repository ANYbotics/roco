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

project(robotControllers)


set(EXECUTABLE_OUTPUT_PATH 	${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH 	${PROJECT_SOURCE_DIR}/lib)


# Noisily default to Release build
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  	message(STATUS "Setting build type to 'Release' as none was specified.")
  	set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  	set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
	"Debug" "Release" "MinSizeRel" "RelWithDebInfo")
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


add_subdirectory(src/common)
add_subdirectory(src/time)

add_library(robotControllers SHARED ${CTRLS_SRCS})
target_link_libraries(robotControllers ${CTRLS_LIBS})


# Add Doxygen documentation
if (BUILD_DOC)
add_subdirectory(doc/doxygen)
endif (BUILD_DOC)



# Generate FindRobotUtils.cmake file
file(WRITE cmake/FindRobotControllers.cmake
"# This file was automatically generated during the installation of the robot utils library
# and can be used through cmake to find the corresponding header files. A copy of this
# file was created in ${CMAKE_ROOT}/Modules (depending on the CMAKE_ROOT variable).

set(RobotControllers_INCLUDE_DIRS
    ${CMAKE_INSTALL_PREFIX}/include/robotControllers/include
)

set(robotControllers_LIBRARIES robotControllers)

set(robotControllers_FOUND TRUE)
"
)


# Setting for make install
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_INSTALL_PREFIX}/include/robotControllers)")
install(DIRECTORY include DESTINATION ${CMAKE_INSTALL_PREFIX}/include/robotControllers)
install(FILES cmake/FindrobotControllers.cmake DESTINATION ${CMAKE_ROOT}/Modules)
install(FILES lib/librobotControllers.so DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

if (BUILD_EXAMPLE)
add_subdirectory(example example)
endif()

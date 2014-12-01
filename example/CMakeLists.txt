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

include_directories(include)
include_directories(../include)

set(EXAMPLE_SRCS
    src/example_main.cpp
    src/ControllerTest.cpp
)

link_directories(${PROJECT_SOURCE_DIR}/lib)

add_executable(example ${EXAMPLE_SRCS})
target_link_libraries(example robotControllers)

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
message("      Including robotControllers/time")


###############
### SOURCES ###
###############
set(CTRLS_SRCS ${CTRLS_SRCS} 
	${CMAKE_CURRENT_SOURCE_DIR}/Time.cpp  
    ${CMAKE_CURRENT_SOURCE_DIR}/TimeStd.cpp  
PARENT_SCOPE)


#################
### LIBRARIES ###
#################
set(CTRLS_LIBS ${CTRLS_LIBS} 

PARENT_SCOPE)
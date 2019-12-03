cmake_minimum_required(VERSION 3.5.1)
project(roco)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 11)
add_compile_options(-Wall -Wextra -Wpedantic)

set(CATKIN_PACKAGE_DEPENDENCIES
  message_logger
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/time/Time.cpp
  src/time/TimeStd.cpp
  src/workers/Worker.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

##########
## Test ##
##########

if(CATKIN_ENABLE_TESTING)
  # Find test dependencies
  catkin_add_gtest(test_${PROJECT_NAME}
    test/test_main.cpp
    test/ControllerExample.cpp
    test/ControllerTest.cpp
    WORKING_DIRECTORY
      ${PROJECT_SOURCE_DIR}/test
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries(test_${PROJECT_NAME}
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
    )

    # Generate test coverage report
    find_package(cmake_code_coverage QUIET)
    if(cmake_code_coverage_FOUND)
      add_gtest_coverage()
    endif(cmake_code_coverage_FOUND)
  endif()
endif()

#################
## Clang Tools ##
#################

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)
cmake_minimum_required(VERSION 2.8.3)

project(${pkg_name})
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roco
${dependencies_1}
)

# Flags
add_definitions(-std=c++11)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES $${PROJECT_NAME}
  CATKIN_DEPENDS
    roco
${dependencies_2}
)

include_directories(include
  $${catkin_INCLUDE_DIRS}
)

add_library($${PROJECT_NAME}
  src/${file_name}.cpp
)

target_link_libraries($${PROJECT_NAME}
  $${catkin_LIBRARIES}
)

add_dependencies($${PROJECT_NAME}
  $${catkin_EXPORTED_TARGETS}
)

install(FILES rocoma_plugin.xml
  DESTINATION $${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS $${PROJECT_NAME}
  ARCHIVE DESTINATION $${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION $${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION $${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/$${PROJECT_NAME}/
  DESTINATION $${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

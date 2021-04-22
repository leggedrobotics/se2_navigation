cmake_minimum_required(VERSION 3.1.3)
project(approach_pose_planner_ros)


set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  approach_pose_planner
  se2_planning_ros
  nav_msgs
  approach_pose_planner_msgs
  se2_visualization_ros
  grid_map_ros
)

set(SRC_FILES
  src/ApproachPosePlannerRos.cpp
  src/loaders.cpp
  src/creators.cpp
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(ompl REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${OMPL_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
    yaml-cpp
  CATKIN_DEPENDS
   ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  yaml-cpp
)

add_executable(approach_pose_planner_node
	src/approach_pose_planner_node.cpp
)
target_link_libraries(approach_pose_planner_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  yaml-cpp
)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#################
## Clang_tools ##
#################
find_package(cmake_clang_tools)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
  add_clang_tooling(
    TARGET ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_FIX
  )
endif(cmake_clang_tools_FOUND)





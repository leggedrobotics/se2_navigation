cmake_minimum_required(VERSION 3.1.3)
project(approach_pose_planner)


set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  se2_planning
)

set(SRC_FILES
  src/ApproachPosePlanner.cpp
  src/ApproachPoseHeuristic.cpp
  src/LineOfSightApproachStateValidator.cpp
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
    ${OMPL_LIBRARIES}
  CATKIN_DEPENDS
   ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    ompl
)

include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
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

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
catkin_add_gtest(${PROJECT_NAME}-test
  test/test_planning.cpp
  test/ApproachPosePlannerTest.cpp
)
endif()

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OMPL_LIBRARIES}
    )
endif()



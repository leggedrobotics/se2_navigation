cmake_minimum_required(VERSION 3.5.1)
project(se2_planning)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRC_FILES
  src/ompl_planner_creators.cpp
  src/GridMapLazyStateValidator.cpp
  src/GridMapStateValidator.cpp
  src/OmplPlanner.cpp
  src/OmplReedsSheppPlanner.cpp
  src/State.cpp
  src/StateValidator.cpp
  src/HeightMap.cpp
)

set(CATKIN_PACKAGE_DEPENDENCIES
  grid_map_core 
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
  set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()

find_package(Boost REQUIRED)
find_package(ompl REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIR}
    test
  LIBRARIES
    ${PROJECT_NAME}
    ${OMPL_LIBRARIES}
  CATKIN_DEPENDS
   ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Eigen3
    Boost
    ompl
)

include_directories(
  include
    test
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
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
  ${Boost_LIBRARIES}
  ${OMPL_LIBRARIES}
)

add_library(test_helpers
  test/test_helpers.cpp
)

target_link_libraries(test_helpers
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

add_executable(planning_benchmark
  src/planning_benchmark.cpp
)
target_link_libraries(planning_benchmark
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${OMPL_LIBRARIES}
  test_helpers
)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ test
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
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
  test/ompl_planner_creators_test.cpp
  test/OmplReedsSheppPlannerTest.cpp
  test/GridMapStateValidatorTest.cpp
  test/GridMapLazyStateValidatorTest.cpp
  )
endif()

if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test 
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    test_helpers)
endif()




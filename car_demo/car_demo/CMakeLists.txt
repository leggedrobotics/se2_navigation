cmake_minimum_required(VERSION 2.8.3)
project(car_demo)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  gazebo_ros
  prius_msgs
  nav_msgs
  se2_navigation_msgs
  pure_pursuit_ros
  tf
)

set (PRIUSCUP_MAJOR_VERSION 0)
set (PRIUSCUP_MINOR_VERSION 1)
set (PRIUSCUP_PATCH_VERSION 0)
set (PRIUSCUP_VERSION_FULL ${PRIUSCUP_MAJOR_VERSION}.${PRIUSCUP_MINOR_VERSION}.${PRIUSCUP_PATCH_VERSION})

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(gazebo 11 REQUIRED)
find_package(ignition-msgs5 REQUIRED)

catkin_package(
 INCLUDE_DIRS 
   include
 LIBRARIES 
   ${PROJECT_NAME}
   yaml-cpp
 CATKIN_DEPENDS 
   ${CATKIN_PACKAGE_DEPENDENCIES}
#  DEPENDS system_lib
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR} 
  ${SDFormat_INCLUDE_DIRS})

add_library(PriusHybridPlugin SHARED plugins/PriusHybridPlugin.cc)
target_link_libraries(PriusHybridPlugin ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES})
add_dependencies(PriusHybridPlugin ${catkin_EXPORTED_TARGETS})


add_library(${PROJECT_NAME}
  src/PriusControllerRos.cpp
  src/PIDController.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(prius_controller_node
 src/prius_controller_node.cpp)
 
target_link_libraries(prius_controller_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
) 
  

catkin_install_python(PROGRAMS nodes/joystick_translator
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS PriusHybridPlugin ${PROJECT_NAME} prius_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

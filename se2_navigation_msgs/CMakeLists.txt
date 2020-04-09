cmake_minimum_required(VERSION 2.8.3)
project(se2_navigation_msgs)

add_definitions(-std=c++11)


set(CATKIN_PACKAGE_DEPENDENCIES
    geometry_msgs
    std_msgs
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  message_generation
)


add_message_files( 
   DIRECTORY 
     msg
)

add_service_files(
	DIRECTORY 
	  srv
)

generate_messages(
    DEPENDENCIES
      ${CATKIN_PACKAGE_DEPENDENCIES}
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
    message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)


cmake_minimum_required(VERSION 2.8.3)
project(approach_pose_planner_msgs)

add_definitions(-std=c++11)


set(CATKIN_PACKAGE_DEPENDENCIES
    geometry_msgs
    std_msgs
    se2_navigation_msgs
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
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


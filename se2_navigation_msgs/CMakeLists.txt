cmake_minimum_required(VERSION 2.8.3)
project(se2_navigation_msgs)

add_definitions(-std=c++11)


set(CATKIN_PACKAGE_DEPENDENCIES
    geometry_msgs
    std_msgs
    tf2
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Eigen3 REQUIRED)

add_message_files( 
   DIRECTORY msg
)

add_service_files(
	DIRECTORY srv
)

generate_messages(
    DEPENDENCIES
      geometry_msgs
      std_msgs
)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES 
   CATKIN_DEPENDS
      message_runtime
      ${CATKIN_PACKAGE_DEPENDENCIES}
   DEPENDS Eigen3 eigen
)


include_directories(include
${catkin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
)


# Mark header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

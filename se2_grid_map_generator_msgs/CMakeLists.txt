cmake_minimum_required(VERSION 2.8.3)
project(se2_grid_map_generator_msgs)

add_definitions(-std=c++11)


set(CATKIN_PACKAGE_DEPENDENCIES
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
        CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        message_runtime
)

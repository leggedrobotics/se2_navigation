cmake_minimum_required(VERSION 2.8.3)
project(se2_grid_map_generator)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SRC_FILES
        src/GridMapGenerator.cpp
    )

set(CATKIN_PACKAGE_DEPENDENCIES
        roscpp
        grid_map_ros
        se2_grid_map_generator_msgs
        )

find_package(catkin REQUIRED
        COMPONENTS
        ${CATKIN_PACKAGE_DEPENDENCIES}
        )

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        CATKIN_DEPENDS
        ${CATKIN_PACKAGE_DEPENDENCIES}
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
        )

add_executable(se2_grid_map_generator_node
        src/se2_grid_map_generator_node.cpp
        )

target_link_libraries(se2_grid_map_generator_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
        )
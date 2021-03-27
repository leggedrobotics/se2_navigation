cmake_minimum_required(VERSION 3.5.1)
project(se2_planning_rviz)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roscpp
  geometry_msgs
  tf2
  visualization_msgs
  rviz
  se2_visualization_ros
  se2_navigation_msgs
  approach_pose_planner_msgs
)

find_package(catkin REQUIRED
  COMPONENTS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)



set(CMAKE_AUTOMOC OFF)
set(HEADER_FILES_QT
  include/se2_planning_rviz/PoseWidget.hpp
  include/se2_planning_rviz/PlanningPanel.hpp
  include/se2_planning_rviz/EditButton.hpp
)

set(SRC_FILES
  src/PlanningPanel.cpp
  src/PoseWidget.cpp
  src/EditButton.cpp
  src/PlanningInteractiveMarkers.cpp
)

set(qt Qt5)
set(qt_include_dirs "")

# This plugin includes Qt widgets, so we must include Qt.
# We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message("Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  # pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
  qt4_wrap_cpp(MOC_FILES
    ${HEADER_FILES_QT}
  )
  set(qt Qt4)
    set(qt_include_dirs ${Qt4Widgets_INCLUDE_DIRS} ${Qt4Gui_INCLUDE_DIRS})
  
else()
  message("Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  # make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
  QT5_WRAP_CPP(MOC_FILES
    ${HEADER_FILES_QT}
  )
  set(qt Qt5)
  set(qt_include_dirs ${Qt5Widgets_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS})
endif()

catkin_package(
  INCLUDE_DIRS
    include
    ${qt_include_dirs}
  LIBRARIES
  	${PROJECT_NAME}
  	${QT_LIBRARIES}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
  	${qt}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${qt_include_dirs}
)


add_definitions(-DQT_NO_KEYWORDS)
add_library(${PROJECT_NAME}
  ${SRC_FILES}
  ${MOC_FILES}
)
target_link_libraries(${PROJECT_NAME} 
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)


install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


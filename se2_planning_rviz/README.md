# se2_planning_rviz
This package builds an rviz plugin that can be used to give high-level goal pose commands to the planner.

## Dependencies

* geometry_msgs 
* roscpp
* se2_visualization_ros (in this repo)
* se2_navigation_msgs (in this repo)
* tf2
* visualization_msgs

This pakcage also depends on [Qt](https://www.qt.io/) version 5.

## Installation
Make sure that you have installed Qt on your machine.
Build with:
`catkin build se2_planning_rviz`

## Usage
Make sure that the package is built and that your workspace has been sourced.

You can add the plugin in a following way:  
1. Open Rviz
2. Click on `Panels` -> `Add New Panel`
3. From the panel type menu, uder tab `se2_planning_rviz`, select `PlanningPanel`
4. The panel should apper on the left of the rviz window.

[<img src="doc/adding_panel.gif" width="547" height="400">](doc/adding_panel.gif)


## Parameters

### Planner ros
This planner extendes the one inside the `se2_planning` package. The parametes are topics to be advertised by the planner.

* `nav_msgs_path_topic` - topic where nav_msgs::Path message is published. Used merely for visualization with Rviz.
* `planning_service_name` - name of the planning service
* `path_msg_topic` - topic where the path will be published (used by the controllers)
* `path_frame` - frame id of the path
* `nav_msg_path_spatial_resolution` - spatial resolution of nav_msgs::Path in meters (euclidean distance between two points in the path)

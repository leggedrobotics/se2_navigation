# approach\_pose\_planner

This package provides an approach pose planner that jointly plans an approach pose and a path. The use case is motivated by an autonomous tree harvesting mission where we know the tree locations (x,y) however it us unclear from which direction should we approach them as the environment gets more unstructured.




## Conventions

### Collision checking
Rectangular collision footprint is shown in an image below. We allow for arbitrary polygons to be used as a collision footprint, as long as the vertices are ordered counter-clockwise. Rectangular fooprints can be created using conveniece functions provided. The footprint is defined by four points:   

* Right Hind (RH)
* Right Front (RF)
* Left Front (LF)
* Left Hind (LH)

User can either specify those point manually, of define 4 measures shown in the image below. The points will then be calculated automatically. All distances are marked with `d` and measured from the origin of the coordinate system.

[<img src="doc/collision_footprint_conventions.png" width="235" height="300">](collision_footprint_conventions.pdf)

## Dependencies

approach_pose_planner depends on following packages:

- [OMPL](https://github.com/ompl/ompl)
- se2_planning

## Installation

Follow the installation guidelines for the [se2_planning](../se2_planning/README.md) package. Once you can build the se2_planning, simply build approach_pose_planner with:

`catkin build approach_pose_planner`  

## Usage
This package provides the algorithm implementation. For ros integration see [approach_pose_planner_ros](../approach_pose_planner_ros/README.md) package.

## Parameters

The approach pose planner reuses some of the algorithms inside the se2_planning package and hece shares some of the parameters

### State Space
See se2_planning.

### Planner
See se2_planning.

### Height Map
* height_layer - Name of the height map layer in the grid map.
* topic - grid map topic in case you a publishing it or loading from a ros bag.

### Robot footprint
* See sketch [here](../se2_planning/README.md);

### Line of Sight Validator
* is_assume_target_always_reachable - bool flag whether to perform reachability analysis as described above.
* collision_checking_area_width - width of the rectangle that is constructed to check the reachability criterion.
* line_of_signt_length_allowed_to_be_in_collision - length of the area that is allowed to be in collision with the target (e.g. if you want to grab stuff).

# pure\_pursuit_core

## Conventions

### Path

Paths are divided into segments. A segment is a part of the path where the driving direction does not change. In the Figure below, the path has three segments. 

[<img src="doc/path_conventions.png" width="300" height="303">](doc/path_conventions.pdf)


### Pure pursuit controller
todo add a sketch with the conventions in the controller

## Installation
Build with:   
`catkin build pure_pursuit_core`   

Run the tests with:   
`catkin build pure_pursuit_core --no-deps --verbose --catkin-make-args run_tests`

## Usage

The code is meant to be used as a part of a bigger project. Please refer to [car_demo](../car_demo) for some examples.

## Dependencies
Other than Eigen, this package has no other dependencies.

## Adding  your own controllers
todo

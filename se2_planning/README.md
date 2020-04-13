# se2_planning

## Installation

se2_planning depends on following packages:

- [OMPL \[v1.4.2\]](https://github.com/ompl/ompl)
- Boost
- Eigen

Eigen and Boost are shipped with the Ubuntu distribution. You can install OMPL from source if you want to use the planner without ROS integration. The easiest wayto build ompl is to build it using catkin build system:

Indice your catkin workspace source folder (src) do:

`git clone git@github.com:ompl/ompl.git`
`catkin build ompl` 

You can also build using CMake, plese refer to https://ompl.kavrakilab.org/.

In case this does not work for you refere to the OMPL [website](https://github.com/ompl/ompl/blob/master/doc/markdown/installation.md) for installation instructions. If you're using the ROS interface, installation can also be done through PPA.

`sudo apt install ros-melodic-ompl`

Build with:

`catkin build se2_planning`

# ecse473_f23_axc1293_ariac_entry

## Package description
This repo is a ROS package that contains the project disign & implementation of ARIAC 2019 for ECSE 473 Modern Robot Programming

## Dependency:
In order to run/view this model, the listed dependencies MUST be satisfied:
-  ROS(Noetic)
-  Ubuntu 20.04
-  ARIAC 2019
-  ecse_373_ariac (ARIAC expansino for Noetic)

## Preliminary Steps
Before lauching gazebo, first need to ensure that ROS(Noetic), ARIAC 2019 and ecse_373_ariac are properlyl installed on a system running Ubuntu 20.04
- Download this package into your catkin workspace `~/<catkin_workspace>/src/`
- Source ecse_373_ariac: `source ~/ecse_373_ariac/devel/setup.bash`
- Build the package with `catkin_make`


### Launch ARIAC 2019

- Source ecse_373_ariac: `source ~/ecse_373_ariac/devel/setup.bash`
- init gazebo `roslaunch ecse_373_ariac ecse_373_ariac.launch`
- start simulation `rosrun ariac_entry ariac_entry_node`

These command will launch the competition

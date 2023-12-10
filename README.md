# ecse473_f23_axc1293_ariac_entry

## Package description
This repo is a ROS package that contains the project disign & implementation of ARIAC 2019 for ECSE 473 Modern Robot Programming, This is a new created repo under the same namespace with the previous one, which was discarded

## Dependency:
In order to run/view this model, the listed dependencies MUST be satisfied:
-  ROS(Noetic)
-  Ubuntu 20.04
-  ARIAC 2019
-  ecse_373_ariac (ARIAC expansino for Noetic)

## Preliminary Steps
Before lauching gazebo, it is necessary to ensure that the following requirements are ALL satisfied:
- ROS(Noetic), ARIAC 2019 and ecse_373_ariac are properlyl installed on a system running Ubuntu 20.04
- Download this package into your catkin workspace `~/<catkin_workspace>/src/`
- Before building this project, make sure to Source ecse_373_ariac fisrt: 
	`source ~/ecse_373_ariac/devel/setup.bash`
- Build the package with `catkin_make`

### Launch ARIAC 2019

- Create terminal A, Source ecse_373_ariac: `source ~/ecse_373_ariac/devel/setup.bash`, and then:
- init gazebo `roslaunch ecse_373_ariac ecse_373_ariac.launch` under terminal A

- Create terminal B, Source the workspace for this project with `source devel/setup.bash`
- start simulation `rosrun ariac_entry ariac_entry_node`

These command will launch the competition

### Theory of Operation

This project contains a report which explains the theory of operation, conbines with the system's block diagram. `Theory of Operation.pdf` can be found at the main repo.

### Known Issue
For unknown reasons, the program is not 100% stable on my laptop, in rare cases, the program finds no possible solution and will crash into something. This situation occurs around 10% during self-testing.

# ecse473_f23_axc1293_ariac_entry
The is a new created repo under the same namespace, the previous old one was discarded because of the missing of tags

## Package description
This new repo is a ROS package that contains the project disign & implementation of ARIAC 2019 for ECSE 473 Modern Robot Programming

### Launch ARIAC 2019

- Source ecse_373_ariac: `source ~/ecse_373_ariac/devel/setup.bash`
- init gazebo `roslaunch ecse_373_ariac ecse_373_ariac.launch`
- start simulation `rosrun ariac_entry ariac_entry_node`

These command will launch the competition


### Lab5

- Setup ARIAC 2019 Environment
- Launch gazebo
- Create basic framework

### Phase 1

- Use `get_trajectory_for_foundation` to correctly locate the location for the foundation
- Use `get_trajectory_for_arm` to correctly locate the angles of the arm
- Do them seperately to avoid collision
- Continuously pick up a part and then drop by setting `switch_grip_status` with true and false
- In this period the program will not tend to ship even if the correct item is picked up, and the for loop will go permenantly
- The whole process do not rely on the previous lab 6 package, which can be found `https://github.com/EricABChen/ecse473_f23_axc1293_ik_service`

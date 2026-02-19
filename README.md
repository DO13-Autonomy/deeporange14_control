# deeporange14_control Package
# Deep Orange DBW Controls Interface Package for Autonomous Driving

### Functionality:
- DeepOrangeDbwCan.cpp: handles the CAN-ROS interface, leveraging SocketCAN, to send commands to Raptor and receive command from Raptor
- DeepOrangeStateSupervisor.cpp: ROS state machine in sync with the Raptor controller state machine
- DeepOrangeInterfaceNode.cpp - instantiates the ROS node for the controller

### Cloning and building:
- Use the following to clone this repository and properly set up submodules:
```
git clone https://github.com/DO13-Autonomy/deeporange14_control.git
cd deeporange14_control
git submodule init
git submodule update
```
  - These commands clone the repository, then initialize and clone the submodules
  - To verify the command has worked, check that `src/pugixml`, `src/raptor-dbw-ros`, and `src/ros_canopen` are not empty
  - See below for more information on these dependencies
- If ROS has been installed and sourced (either in `~/.bashrc` or manually), run the following from the top-level of the repository to install dependencies:
```
rosdep install --from-paths src --ignore-src -r -y 
```
- Build the code by calling `catkin build` in the top-level of the repo

### How to Use:
- To add/update CAN msg IDs- update these in the dbc in the `dbc` folder and `DeepOrangeCanDispatch.h`
- State machine enumerations are given in `DeepOrangeStateEnums.h`

### External Dependencies:
- `can_dbc_parser`: Developed by New Eagle as part of metapackage - https://github.com/NewEagleRaptor/raptor-dbw-ros
- `socketcan_bridge` : Package for interfacing CAN to ROS on Linux - https://github.com/ros-industrial/ros_canopen
- `can_msgs` : Package to support CAN msg format on ROS topics. - https://github.com/ros-industrial/ros_canopen
- `pugixml`: Package for processing XML files. - https://github.com/joselusl/pugixml

### Testing:
Automatic linting capability was added to the deeporange14_control package through `roslint` and `roslaunch-check`.  After building the pacakge, the tests can be launched using
```
catkin test deeporange14_control
```
Results will be printed to the console and in associated XML files in `build/deeporange14_control/test_results/deeporange14_control`.

The third-party pacakges also contain some unit tests which can be launched one-at-a-time using the above command (substituting the package name).  All tests in the workspace can be launched with `catkin test`.

Additional linting can be done with the [catkin_lint](https://fkie.github.io/catkin_lint/) tool, which can be installed following the instructions at the link and launched from the root of the catkin workspace using
```
source devel/setup.bash
catkin_lint --pkg deeporange14_control
```

### Contributors:
 - Pavan Kumar Athinarapu (athinak@clemson.edu)
 - Anirudh Kyatham (akyata@clemson.edu)
 - Mrunali Mahadeo Sawant (msawant@clemson.edu)
 - Vasanth Seethapathi (vasants@clemson.edu)
 - Sanskruti Jadhav (sanskrj@clemson.edu)
 - Shubham Gupta (gupta9@clemson.edu)
 - Shubhankar Kulkarni (sskulka@clemson.edu)
 - Priyanshu Rawat (prawat@clemson.edu)
 - Vasudev Purohit (vpurohi@clemson.edu)

 - Program Manager/Maintainer: Chris Paredis (paredis@clemson.edu)

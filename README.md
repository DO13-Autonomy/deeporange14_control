# deeporange14_control Package
# Deep Orange DBW Controls Interface Package for Autonomous Driving

### Functionality:

- DeepOrangeDbwCan.cpp: Enable passing the ROS topics to the Raptor DBW controller via SocketCAN, which executes the commands for the platform to move. Additionally monitors rtk fix, logging status
- VelocityController.cpp: Use the vehicle dynamics model to convert vehicle velocity to left and right track velocity/torques.
- StateMachine.cpp: A ROS state machine in sync with the Raptor controller state machine.
- RosHealthMonitor.cpp: A node that Monitor health of raptor hanshake, brake acknowledgement from raptor , autonomy stack fault and dbw ros mode
- DataLoggerNode.cpp: logging CAN and ROS data while monitoring logging status active or Fail
- Node.cpp - Instantiates the three objects in a single node (as of now).

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
- Install the [Kvaser interface package](https://github.com/astuff/kvaser_interface) with the following steps from the `README.md` for that repo:
```
sudo apt-add-repository ppa:astuff/kvaser-linux 
sudo apt update 
sudo apt install kvaser-canlib-dev kvaser-drivers-dkms 

sudo apt install apt-transport-https 
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list' 
sudo apt update 
sudo apt install ros-noetic-kvaser-interface 
```
- Build the code by calling `catkin_make` in the top-level of the repo


### How to Use:
- To add/update CAN msg IDs- update these in the dbc in the `dbc` folder as well as `dispatch_can_msgs.h` include file
- Vehicle and subsystem states are enumerated in `DeeporangeStateEnums.h`


### Dependencies:
- `can_dbc_parser`: Developed by New Eagle as part of metapackage - https://github.com/NewEagleRaptor/raptor-dbw-ros
- `deeporange14_msgs` : Package for custom messages for DO13 DBW operation
- `socketcan_bridge` : Package for interfacing CAN to ROS on Linux - https://github.com/ros-industrial/ros_canopen
- `can_msgs` : Package to support CAN msg format on ROS topics. - https://github.com/ros-industrial/ros_canopen
- 'pugixml': Package for processing XML files. - https://github.com/joselusl/pugixml


### Testing:
Automatic linting capability was added to the deeporange14_control package through `roslint` and `roslaunch-check`.  After building the pacakge, the tests can be launched using
```
catkin test deeporange14_control
```
Results will be printed to the console and in associated XML files in `build/deeporange14_control/test_results/deeporange14_control`.

The third-party pacakges also contain some unit tests which can be launched one-at-a-time using the above command (substituting the package name).  All tests in the workspace can be launched with `catkin test`.

Additional linting can be done with the [catkin_lint](https://fkie.github.io/catkin_lint/) tool, which can be installed following the instructions at the link and launched from the root of the catkin workspace using
```
catkin_lint --pkg deeporange14_control
```


### Contributors:

 - Sanskruti Jadhav (sanskrj@clemson.edu)
 - Shubham Gupta (gupta9@clemson.edu)
 - Shubhankar Kulkarni (sskulka@clemson.edu)
 - Priyanshu Rawat (prawat@clemson.edu)
 - Vasudev Purohit (vpurohi@clemson.edu)

 - Program Manager/Maintainer: Chris Paredis (paredis@clemson.edu)

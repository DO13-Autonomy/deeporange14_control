# deeporange14_control Package
# Deep Orange DBW Controls Interface Package for Autonomous Driving

### Functionality:

- DeepOrangeDbwCan.cpp: Enable passing the ROS topics to the Raptor DBW controller via SocketCAN, which executes the commands for the platform to move. Additionally monitors rtk fix, logging status
- VelocityController.cpp: Use the vehicle dynamics model to convert vehicle velocity to left and right track velocity/torques.
- StateMachine.cpp: A ROS state machine in sync with the Raptor controller state machine.
- RosHealthMonitor.cpp: A node that Monitor health of raptor hanshake, brake acknowledgement from raptor , autonomy stack fault and dbw ros mode
- DataLoggerNode.cpp: logging CAN and ROS data while monitoring logging status active or Fail
- Node.cpp - Instantiates the three objects in a single node (as of now).

### How to Use:
- To add/update CAN msg IDs- update these in the dbc in the `dbc` folder as well as `dispatch_can_msgs.h` include file
- Vehicle and subsystem states are enumerated in `DeeporangeStateEnums.h`


### Dependencies:
- `can_dbc_parser`: Developed by New Eagle as part of metapackage - https://github.com/NewEagleRaptor/raptor-dbw-ros
- `deeporange14_msgs` : Package for custom messages for DO13 DBW operation
- `socketcan_bridge` : Package for interfacing CAN to ROS on Linux - https://github.com/ros-industrial/ros_canopen
- `can_msgs` : Package to support CAN msg format on ROS topics. - https://github.com/ros-industrial/ros_canopen
- 'pugixml': Package for processing XML files. - https://github.com/joselusl/pugixml
- rosdep install --from-paths src --ignore-src -r -y 
  

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

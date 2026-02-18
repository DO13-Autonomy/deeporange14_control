/*
Implement a controller node in the Deep Orange 14 drive-by-wire package

Uses the New Eagle package can_dbc_parser, included as a submodule:
(https://github.com/NewEagleRaptor/raptor-dbw-ros/tree/master/can_dbc_parser)
*/

#include <ros/ros.h>

#include<deeporange14_control/DeepOrangeStateSupervisor.h>
#include<deeporange14_control/DeepOrangeDbwCan.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "DeepOrangeInterface");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  // create state machine object
  deeporange14::DeepOrangeStateSupervisor deeporange_stateSupervisor(nh, priv_nh);

  // create ROS-CAN interface object
  deeporange14::DeepOrangeDbwCan deeporange_canNode(nh, priv_nh);

  ros::spin();

  return 0;
}

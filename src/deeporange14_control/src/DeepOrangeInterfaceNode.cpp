/*********************************************************************
Written for use in Deep Orange 14 Drive-by-wire package
Makes use of New Eagle package can_dbc_parser: (https://github.com/NewEagleRaptor/raptor-dbw-ros/tree/master/can_dbc_parser)
#include <deeporange14_control/DeepOrangeDbwCan.h>
#include <deeporange14_control/DataLogger.h>
 *********************************************************************/

#include <ros/ros.h>

#include<deeporange14_control/DeepOrangeStateSupervisor.h>
#include<deeporange14_control/DeepOrangeVelocityController.h>
#include<deeporange14_control/DataLogger.h>
#include<deeporange14_control/DeepOrangeDbwCan.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "DeepOrangeInterface");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  // create StateMachine object
  deeporange14::DeepOrangeStateSupervisor deeporange_stateSupervisor(nh, priv_nh);
  deeporange14::VelocityController deeporange_velocityController(nh, priv_nh);

  // create Data Logger object
  deeporange14::DataLogger deeporange_datalogger(nh, priv_nh);
  deeporange14::DeepOrangeDbwCan deeporange_canNode(nh, priv_nh);

  ros::spin();

  return 0;
}

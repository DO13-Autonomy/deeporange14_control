/*********************************************************************
Written for use in Deep Orange 14 Drive-by-wire package
Makes use of New Eagle package can_dbc_parser: (https://github.com/NewEagleRaptor/raptor-dbw-ros/tree/master/can_dbc_parser)
#include <deeporange14_control/DeepOrangeDbwCan.h>
#include <deeporange14_control/DataLogger.h>
 *********************************************************************/

#include <rclcpp/rclcpp.hpp>

#include<deeporange14_control/DeepOrangeStateSupervisor.h>
#include<deeporange14_control/DeepOrangeVelocityController.h>
//#include<deeporange14_control/DataLogger.h>
#include<deeporange14_control/DeepOrangeDbwCan.h>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("deep_orange_dbw_can");
  //ros::NodeHandle nh;
  //ros::NodeHandle priv_nh("~");

  // create StateMachine object
  deeporange14::DeepOrangeStateSupervisor deeporange_stateSupervisor(node);
  deeporange14::VelocityController deeporange_velocityController(node);

  // create Data Logger object
  //deeporange14::DataLogger deeporange_datalogger(node, priv_nh);
  deeporange14::DeepOrangeDbwCan deeporange_canNode(node);

  rclcpp::spin(node);

  return 0;
}

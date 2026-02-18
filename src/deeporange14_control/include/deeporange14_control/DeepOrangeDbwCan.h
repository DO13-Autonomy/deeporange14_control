/* 
A class to manage can msgs (parse and publish to can_tx/rx) using dbc file provided
Relies on DbwSupervisor to figure out what to send to Raptor. Uses "ros_state" topic to get this info
Receives CAN data from socketcan node and provides info to DbwSupervisor
*/

#ifndef DEEPORANGE_DBW_CAN_H_
#define DEEPORANGE_DBW_CAN_H_

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <can_msgs/Frame.h>

#include <can_dbc_parser/DbcMessage.h>
#include <can_dbc_parser/DbcSignal.h>
#include <can_dbc_parser/Dbc.h>
#include <can_dbc_parser/DbcBuilder.h>

#include <deeporange14_control/DeepOrangeCanDispatch.h>
#include <deeporange14_msgs/AutonomyCommandMsg.h>
#include <deeporange14_msgs/AutonomyMeasurementMsg.h>

namespace deeporange14 {
class DeepOrangeDbwCan {
 public:
  DeepOrangeDbwCan(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
  ~DeepOrangeDbwCan();

 private:
  void recvMeasFromCan(const can_msgs::Frame::ConstPtr& msg);
  void pubCmdToCan(const deeporange14_msgs::AutonomyCommandMsg& msg);

  // publishers
  ros::Publisher pub_can_;
  ros::Publisher pub_auMeas_;

  // subscribers
  ros::Subscriber sub_can_;
  ros::Subscriber sub_auCmd_;

  // relevant message objects
  deeporange14_msgs::AutonomyCommandMsg auCmdMsg_;
  deeporange14_msgs::AutonomyMeasurementMsg auMeasMsg_;

  // for handling CAN frames
  std::string frameId_;
  can_msgs::Frame frame_;

  // for DBC file processing
  NewEagle::Dbc autonomyDbc_;
  std::string dbcFile_;
  
  // heartbeat for autonomy stack
  int ros_hb_ = 0;

  // TODO - make this a parameter
  std::string topic_ns = "/deeporange1314";
};
}  // namespace deeporange14

#endif  // DEEPORANGE_DBW_CAN_H_

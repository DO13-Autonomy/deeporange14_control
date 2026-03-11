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
#include <deeporange14_control/DeepOrangeStateEnums.h>
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

  void sendDefaultMeasMsg(const ros::TimerEvent& event);

  // publishers
  ros::Publisher pub_can_;
  ros::Publisher pub_auMeas_;

  // subscribers
  ros::Subscriber sub_can_;
  ros::Subscriber sub_auCmd_;

  // relevant message objects
  deeporange14_msgs::AutonomyMeasurementMsg au_meas_msg_;

  // for handling CAN frames
  can_msgs::Frame frame_;

  // for DBC file processing
  NewEagle::Dbc autonomy_dbc_;

  // heartbeat for autonomy stack
  int ros_hb_ = 0;

  // timer for Raptor timeout
  ros::Timer raptor_meas_timer_;

  // variables to hold parameters
  std::string topic_ns_;
  float raptor_meas_timeout_s_;
  std::string topic_au_cmd_;
  std::string topic_au_meas_;
  std::string topic_from_can_;
  std::string topic_to_can_;
  std::string dbc_file_;
};
}  // namespace deeporange14

#endif  // DEEPORANGE_DBW_CAN_H_

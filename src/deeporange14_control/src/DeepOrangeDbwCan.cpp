/* 
Manage the CAN-ROS interface using a specified DBC file to read from or write to the CAN bus

This file only acts to traslate between ROS and CAN messages by directly transferring values from
the ROS message into the CAN frame (no additional processing is done)
*/

#include <string>

#include <deeporange14_control/DeepOrangeDbwCan.h>

namespace deeporange14 {
DeepOrangeDbwCan::DeepOrangeDbwCan(ros::NodeHandle &node, ros::NodeHandle &priv_nh) {
  /* get parameters from launch file */
  priv_nh.getParam("vehicle_ns", topic_ns_);  // namespace for vehicle topics
  priv_nh.getParam("au_cmd_topic", topic_au_cmd_);  // topic for AutonomyCommand message (pub)
  priv_nh.getParam("au_meas_topic", topic_au_meas_);  // topic for AutonomyMeasurement message (sub)
  priv_nh.getParam("ros_from_can", topic_from_can_);  // topic from SocketCAN
  priv_nh.getParam("can_from_ros", topic_to_can_);  // topic to SocketCAN
  priv_nh.getParam("dbc_file", dbc_file_);  // DBC filename

  /* subscribers and publishers */

  // ---------- socketcan_bridge ---------- //
  // topic names are defined in control.launch
  sub_can_ = node.subscribe(topic_from_can_,
                            10,
                            &DeepOrangeDbwCan::recvMeasFromCan,
                            this,
                            ros::TransportHints().tcpNoDelay(true));

  pub_can_ = node.advertise<can_msgs::Frame>(topic_to_can_, 10);

  // --------------- to ROS --------------- //
  pub_auMeas_ = node.advertise<deeporange14_msgs::AutonomyMeasurementMsg>(
    std::string(topic_ns_ + "/" + topic_au_meas_), 10);

  // -------------- from ROS -------------- //
  sub_auCmd_ = node.subscribe(std::string(topic_ns_ + "/" + topic_au_cmd_),
                              10,
                              &DeepOrangeDbwCan::pubCmdToCan,
                              this,
                              ros::TransportHints().tcpNoDelay(true));

  // instantiate DBC object
  autonomy_dbc_ = NewEagle::DbcBuilder().NewDbc(dbc_file_);
}

DeepOrangeDbwCan::~DeepOrangeDbwCan() {}

void DeepOrangeDbwCan::recvMeasFromCan(const can_msgs::Frame::ConstPtr& msg) {
  ROS_DEBUG("CAN message received, ID = %d. is_rtr: %d, is_error: %d", msg->id, msg->is_rtr, msg->is_error);

  // if the frame contains data and is not in error, check the ID and extract the data if it matches
  if (!msg->is_rtr && !msg->is_error) {
    if (msg->id == ID_AUTONOMY_MEAS) {
      // Extract measurments and Raptor state
      NewEagle::DbcMessage* message = autonomy_dbc_.GetMessageById(ID_AUTONOMY_MEAS);

      if (msg->dlc >= message->GetDlc()) {
        message->SetFrame(msg);
        auMeasMsg_.header.stamp = msg->header.stamp;

        // the assumption here is that clamping/scaling is done on the receiving end
        auMeasMsg_.vx_meas = message->GetSignal("vx_meas")->GetResult();
        auMeasMsg_.curv_meas = message->GetSignal("curv_meas")->GetResult();
        auMeasMsg_.dbw_state = message->GetSignal("dbw_state")->GetResult();
        auMeasMsg_.dbw_seq = message->GetSignal("dbw_state")->GetResult();
        pub_auMeas_.publish(auMeasMsg_);
      }
    }
  }
}

void DeepOrangeDbwCan::pubCmdToCan(const deeporange14_msgs::AutonomyCommandMsg& msg) {
  // build CAN frame from commands and state
  NewEagle::DbcMessage* message = autonomy_dbc_ .GetMessageById(ID_AUTONOMY_CMD);

  // increment the sequencer for the autonomy message from 0 to 255 (limites defined in message file)
  if (ros_hb_ < msg.SEQ_MAX) ros_hb_++;
  else ros_hb_ = msg.SEQ_MIN;

  // the assumption here is that clamping/scaling is done on the sending end
  message->GetSignal("vx_cmd")->SetResult(msg.vx_cmd);
  message->GetSignal("curv_cmd")->SetResult(msg.curv_cmd);
  message->GetSignal("au_state")->SetResult(msg.au_state);
  message->GetSignal("au_seq")->SetResult(ros_hb_);

  frame_ = message->GetFrame();
  pub_can_.publish(frame_);
}
}  // namespace deeporange14

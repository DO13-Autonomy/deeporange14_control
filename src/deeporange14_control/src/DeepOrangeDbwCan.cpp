/* 
A class to manage can msgs (parse and publish to can_tx/rx) using dbc file provided
Relies on DbwSupervisor to figure out what to send to Raptor. Uses "ros_state" topic to get this info
Receives CAN data from socketcan node and provides info to DbwSupervisor
*/

#include <string>

#include <deeporange14_control/DeepOrangeDbwCan.h>

namespace deeporange14  // TODO -> change namespace
{
DeepOrangeDbwCan::DeepOrangeDbwCan(ros::NodeHandle &node, ros::NodeHandle &priv_nh)
{
  /* Instantiating subscribers and publishers */

  sub_can_ = node.subscribe("can_tx", 10, &DeepOrangeDbwCan::recvCAN, this, ros::TransportHints().tcpNoDelay(true));
  pub_can_ = node.advertise<can_msgs::Frame>("can_rx", 10);

  // --------------- To ROS --------------- //

  // Raptor state & brake status
  pub_raptorState_ = node.advertise<deeporange14_msgs::RaptorStateMsg>(std::string(topic_ns +"/raptor_state"), 10);

  // --------------- To CAN --------------- //

  // Measured velocities
  pub_auStatus_ = node.advertise<deeporange14_msgs::AuStatusMsg>(std::string(topic_ns + "/autonomy_log_status"), 10);
  sub_odom_ = node.subscribe("/novatel/oem7/odom", 10, &DeepOrangeDbwCan::getMeasuredVx, this, ros::TransportHints().tcpNoDelay(true));
  sub_gpsImu_ = node.subscribe("/gps/imu", 10, &DeepOrangeDbwCan::getMeasuredWz, this, ros::TransportHints().tcpNoDelay(true));
  sub_autonomyLog_ = node.subscribe(std::string(topic_ns + "/autonomy_log_status"), 10, &DeepOrangeDbwCan::publishAuStatustoCAN, this, ros::TransportHints().tcpNoDelay(true));

  // Rtk & log status
  sub_rtk_ = node.subscribe("/novatel/oem7/inspvax", 10, &DeepOrangeDbwCan::getRtkStatus, this, ros::TransportHints().tcpNoDelay(true));

  // AU State, Torque & Brake cmds
  sub_auMobility_ = node.subscribe(std::string(topic_ns + "/cmd_mobility"), 10, &DeepOrangeDbwCan::publishCommandstoCAN, this, ros::TransportHints().tcpNoDelay(true));  // TODO -> RENAME TOPIC

  // Setting up a timer
  timer_ = node.createTimer(ros::Duration(1.0 / 50.0), &DeepOrangeDbwCan::publishAuStatus, this);

  // Get dbc param values
  priv_nh.getParam("dbc_file_ros", dbcFileRos_);
  priv_nh.getParam("dbc_file_raptor", dbcFileRaptor_);

  // Instantiate the dbc class
  rosDbc_ = NewEagle::DbcBuilder().NewDbc(dbcFileRos_);
  raptorDbc_ = NewEagle::DbcBuilder().NewDbc(dbcFileRaptor_);
}

DeepOrangeDbwCan::~DeepOrangeDbwCan() {}

void DeepOrangeDbwCan::recvCAN(const can_msgs::Frame::ConstPtr& msg)
{
  // ROS_WARN("Inside RecvCan. is_rtr: %d, is_error: %d",msg->is_rtr, msg->is_error);
  if (!msg->is_rtr && !msg->is_error)
  {
    switch (msg->id)
    {
      // Getting brake status
      case ID_VD_Brake_Msg:
      {
        NewEagle::DbcMessage* message = raptorDbc_.GetMessageById(ID_VD_Brake_Msg);
        if (msg->dlc >= message->GetDlc())
        {
          message->SetFrame(msg);
          raptorMsg_.header.stamp = msg->header.stamp;
          raptorMsg_.brk_Rpres = message->GetSignal("brk_ip_Rpres")->GetResult();
          raptorMsg_.brk_Lpres = message->GetSignal("brk_ip_Lpres")->GetResult();
        }
      }
      break;

      // Getting raptor state and dbw mode
      case ID_Raptor_Main_Msg:
      {
        NewEagle::DbcMessage* message = raptorDbc_.GetMessageById(ID_Raptor_Main_Msg);
        if (msg->dlc >= message->GetDlc())
        {
          message->SetFrame(msg);
          raptorMsg_.system_state = message->GetSignal("SYS_STATE")->GetResult();
          raptorMsg_.dbw_mode = message->GetSignal("DBW_MODE")->GetResult();
          raptorMsg_.speed_state = message->GetSignal("spdSt")->GetResult();
          raptorMsg_.log_cmd = message->GetSignal("log_cmd")->GetResult();
          pub_raptorState_.publish(raptorMsg_);
        }
      }
      break;
    }
  }
}

void DeepOrangeDbwCan::publishCommandstoCAN(const deeporange14_msgs::MobilityMsg& msg)
// Get AU state, Torque & Brake commands
{
  NewEagle::DbcMessage* message = rosDbc_ .GetMessageById(ID_AU_CONTROL_MSG);
  // Increamenting Ros heartbeat by `1` until `15`
  if (*ros_hb_ptr_ < 15) (*ros_hb_ptr_)++;
  else *ros_hb_ptr_ = 1;

  message->GetSignal("auSt")->SetResult(msg.au_state);
  message->GetSignal("au_heartbeat")->SetResult(*ros_hb_ptr_);
  message->GetSignal("tqL_cmd")->SetResult(msg.tqL_cmd);
  message->GetSignal("tqR_cmd")->SetResult(msg.tqR_cmd);

  frame_ = message->GetFrame();
  pub_can_.publish(frame_);
}

// Publishing measured velocities, rtk and logging status to CAN
void DeepOrangeDbwCan::publishAuStatustoCAN(const deeporange14_msgs::AuStatusMsg& msg)
{
  NewEagle::DbcMessage* message = rosDbc_.GetMessageById(ID_AU_STATUS_MSG);
  // Get current time
  ros::param::get("/log_status", log_st);
  double currTime = ros::Time::now().toSec();
  // Set values if they are less than 2 seconds old otherwise `0`
  if (currTime - msg.timesecVx < 2) message->GetSignal("meas_gps_lin")->SetResult(msg.measuredVx);
  else message->GetSignal("meas_gps_lin")->SetResult(0);

  if (currTime - msg.timesecWz < 2) message->GetSignal("meas_gps_ang")->SetResult(msg.measuredWz);
  else message->GetSignal("meas_gps_ang")->SetResult(0);

  if (currTime - msg.timesecRtk < 2) message->GetSignal("rtk_ack")->SetResult(msg.rtkStatus);
  else message->GetSignal("rtk_ack")->SetResult(0);

  // Getting logging status using param value set by data logger
  if (ros::param::has("/log_status"))
  {
    // If param has value then get it otherwise send `0`
    ros::param::get("/log_status", log_st);
    message->GetSignal("log_ack")->SetResult(log_st);
  }
  else message->GetSignal("log_ack")->SetResult(0);

  frame_ = message->GetFrame();
  pub_can_.publish(frame_);
}

// Publish AU status at 50hz
void DeepOrangeDbwCan::publishAuStatus(const ros::TimerEvent& event)
{
  auStatusMsg_.header.stamp = ros::Time::now();
  // Updating values
  auStatusMsg_.rtkStatus = *rtk_status_ptr_;
  auStatusMsg_.measuredVx = *measVx_ptr_;
  auStatusMsg_.measuredWz = *measWz_ptr_;
  // Updating time stamp
  auStatusMsg_.timesecVx = *time_Vx_ptr_;
  auStatusMsg_.timesecWz = *time_Wz_ptr_;
  auStatusMsg_.timesecRtk = *time_Rtk_ptr_;
  // Publishing values
  pub_auStatus_.publish(auStatusMsg_);
}

// Getting RTK status
void DeepOrangeDbwCan::getRtkStatus(const novatel_oem7_msgs::INSPVAX& msg)
{
  *time_Rtk_ptr_ = msg.header.stamp.sec + msg.header.stamp.nsec*1e-9;
  if (msg.pos_type.type == 50 || msg.pos_type.type == 56)
  {
    *rtk_status_ptr_ = 1;
  }
  else
  {
    *rtk_status_ptr_ = 0;
  }
}

// Getting Measured Wz
void DeepOrangeDbwCan::getMeasuredWz(const sensor_msgs::Imu& msg)
{
  *time_Wz_ptr_ = msg.header.stamp.sec + msg.header.stamp.nsec*1e-9;
  vectorWz_.push_back(msg.angular_velocity.z);

  if (vectorWz_.size() == 4)
  {
    for (int i = 0; i < 4; i++)
    {
      averageWz_ = averageWz_ + vectorWz_[i];
    }

    averageWz_ = averageWz_/4;
    // Publishing the average of 4 imu values
    *measWz_ptr_ = averageWz_;
    vectorWz_.clear();
  }
  averageWz_ = 0;
}

// Getting Measured Vx
void DeepOrangeDbwCan::getMeasuredVx(const nav_msgs::Odometry& msg)
{
  *time_Vx_ptr_ = msg.header.stamp.sec + msg.header.stamp.nsec*1e-9;
  vectorVx_.push_back(msg.twist.twist.linear.x);

  if (vectorVx_.size( ) == 2)
  {
    for (int i = 0; i < 2; i++)
    {
      averageVx_ = averageVx_ + vectorVx_[i];
    }

    averageVx_ = averageVx_/2;
    // Publishing the average of 2 odom values
    *measVx_ptr_ = averageVx_;
    vectorVx_.clear();
  }
  averageVx_ = 0;
}
}  // namespace deeporange14

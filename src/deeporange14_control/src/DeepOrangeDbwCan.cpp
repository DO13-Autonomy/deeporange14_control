/* 
A class to manage can msgs (parse and publish to can_tx/rx) using dbc file provided
Relies on DbwSupervisor to figure out what to send to Raptor. Uses "ros_state" topic to get this info
Receives CAN data from socketcan node and provides info to DbwSupervisor
*/

#include <deeporange14_control/DeepOrangeDbwCan.h>

namespace deeporange14 {  // TODO -> change namespace
DeepOrangeDbwCan::DeepOrangeDbwCan(rclcpp::Node::SharedPtr node) : node_(node) { //}, ros::NodeHandle &priv_nh) {
  /* Instantiating subscribers and publishers */
  sub_can_ = node->create_subscription<can_msgs::msg::Frame>("can_tx", 10, std::bind(&DeepOrangeDbwCan::recvCAN, this, std::placeholders::_1)); // TODO - QOS settings to match ros::TransportHints().tcpNoDelay(true)
  pub_can_ = node->create_publisher<can_msgs::msg::Frame>("can_rx", 10);

  // --------------- To ROS --------------- //
  // Raptor state & brake status
  pub_raptorState_ = node->create_publisher<deeporange14_msgs::msg::RaptorState>(std::string(topic_ns +"/raptor_state"), 10);

  // --------------- To CAN --------------- //
  // Measured velocities
  pub_auStatus_ = node->create_publisher<deeporange14_msgs::msg::AuStatus>(std::string(topic_ns + "/autonomy_log_status"), 10);
  sub_odom_ = node->create_subscription<nav_msgs::msg::Odometry>("/novatel/oem7/odom", 10, std::bind(&DeepOrangeDbwCan::getMeasuredVx, this, std::placeholders::_1)); // TODO - QOS settings to match ros::TransportHints().tcpNoDelay(true)
  sub_gpsImu_ = node->create_subscription<sensor_msgs::msg::Imu>("/gps/imu", 10, std::bind(&DeepOrangeDbwCan::getMeasuredWz, this, std::placeholders::_1)); // TODO - QOS settings to match ros::TransportHints().tcpNoDelay(true)
  sub_autonomyLog_ = node->create_subscription<deeporange14_msgs::msg::AuStatus>(std::string(topic_ns + "/autonomy_log_status"), 10,
                                    std::bind(&DeepOrangeDbwCan::publishAuStatustoCAN, this, std::placeholders::_1)); // TODO - QOS settings to match ros::TransportHints().tcpNoDelay(true)

  // Rtk & log status
  sub_rtk_ = node->create_subscription<novatel_oem7_msgs::msg::INSPVAX>("/novatel/oem7/inspvax", 10, std::bind(&DeepOrangeDbwCan::getRtkStatus, this, std::placeholders::_1)); // TODO - QOS settings to match ros::TransportHints().tcpNoDelay(true)

  // AU State, Torque & Brake cmds
  sub_auMobility_ = node->create_subscription<deeporange14_msgs::msg::Mobility>(std::string(topic_ns + "/cmd_mobility"), 10, 
                            std::bind(&DeepOrangeDbwCan::publishCommandstoCAN, this, std::placeholders::_1)); // TODO - QOS settings to match ros::TransportHints().tcpNoDelay(true),  TODO (old) -> RENAME TOPIC

  // Setting up a timer
  float timer_period = 1.0 / 50;
  timer_ = node->create_wall_timer(std::chrono::duration<double, std::nano>(timer_period), std::bind(&DeepOrangeDbwCan::publishAuStatus, this));

  // Get dbc param values
  node->get_parameter("dbc_file_ros", dbcFileRos_);
  node->get_parameter("dbc_file_raptor", dbcFileRaptor_);

  readDbcFile(dbcFileRos_, dbcRosRaw_);
  readDbcFile(dbcFileRaptor_, dbcRaptorRaw_);

  // Instantiate the dbc class
  rosDbc_ = NewEagle::DbcBuilder().NewDbc(dbcRosRaw_);
  raptorDbc_ = NewEagle::DbcBuilder().NewDbc(dbcRaptorRaw_);
}

DeepOrangeDbwCan::~DeepOrangeDbwCan() {}

// TODO - this needs to be tested (it seems to work external to this package/ROS, but not sure if it will also work within the node)
// function to read the contents of the DBC files, which are needed by the NewDbc function, but ROS2 doesn't allow textfile parameters
void readDbcFile(std::string &dbc_file, std::string &dbc_raw) {
  std::stringstream dbc_stream;
  std::filestream fs{dbc_file, fs.in};

  RCLCPP_INFO(node_->get_logger(), "Reading from DBC file: " + dbc_file);

  if (!is.is_open()) {
    RCLCPP_ERROR(node_->get_logger(), "Unable to open DBC file " + dbc_file);
    dbc_raw = "";
  }
  else {
    dbc_stream << fs.rdbuf();
    dbc_raw = dbc_stream.str();
  }
}

void DeepOrangeDbwCan::recvCAN(const can_msgs::msg::Frame::SharedPtr msg) {
  // ROS_WARN("Inside RecvCan. is_rtr: %d, is_error: %d",msg->is_rtr, msg->is_error);
  if (!msg->is_rtr && !msg->is_error) {
    switch (msg->id) {
      // Getting brake status
      case ID_VD_Brake_Msg: {
        NewEagle::DbcMessage* message = raptorDbc_.GetMessageById(ID_VD_Brake_Msg);
        if (msg->dlc >= message->GetDlc()) {
          message->SetFrame(msg);
          raptorMsg_.header.stamp = msg->header.stamp;
          raptorMsg_.brk_rpres = message->GetSignal("brk_ip_Rpres")->GetResult();
          raptorMsg_.brk_lpres = message->GetSignal("brk_ip_Lpres")->GetResult();
        }
      }
      break;

      // Getting raptor state and dbw mode
      case ID_Raptor_Main_Msg: {
        NewEagle::DbcMessage* message = raptorDbc_.GetMessageById(ID_Raptor_Main_Msg);
        if (msg->dlc >= message->GetDlc()) {
          message->SetFrame(msg);
          raptorMsg_.system_state = message->GetSignal("SYS_STATE")->GetResult();
          raptorMsg_.dbw_mode = message->GetSignal("DBW_MODE")->GetResult();
          raptorMsg_.speed_state = message->GetSignal("spdSt")->GetResult();
          raptorMsg_.log_cmd = message->GetSignal("log_cmd")->GetResult();
          this->pub_raptorState_->publish(raptorMsg_);
        }
      }
      break;
    }
  }
}

void DeepOrangeDbwCan::publishCommandstoCAN(const deeporange14_msgs::msg::Mobility& msg) {
// Get AU state, Torque & Brake commands
  NewEagle::DbcMessage* message = rosDbc_ .GetMessageById(ID_AU_CONTROL_MSG);
  // Increamenting Ros heartbeat by `1` until `15`
  if (*ros_hb_ptr_ < 15) (*ros_hb_ptr_)++;
  else *ros_hb_ptr_ = 1;

  message->GetSignal("auSt")->SetResult(msg.au_state);
  message->GetSignal("au_heartbeat")->SetResult(*ros_hb_ptr_);
  message->GetSignal("tqL_cmd")->SetResult(msg.tql_cmd);
  message->GetSignal("tqR_cmd")->SetResult(msg.tqr_cmd);

  frame_ = message->GetFrame();
  this->pub_can_->publish(frame_);
}

// Publishing measured velocities, rtk and logging status to CAN
void DeepOrangeDbwCan::publishAuStatustoCAN(const deeporange14_msgs::msg::AuStatus& msg) {
  NewEagle::DbcMessage* message = rosDbc_.GetMessageById(ID_AU_STATUS_MSG);
  // Get current time
  //log_st = this->get_parameter("/log_status").as_int();
  double currTime = node_->get_clock()->now().seconds();
  // Set values if they are less than 2 seconds old otherwise `0`
  if (currTime - msg.timesec_vx < 2) message->GetSignal("meas_gps_lin")->SetResult(msg.measured_vx);
  else message->GetSignal("meas_gps_lin")->SetResult(0);

  if (currTime - msg.timesec_wz < 2) message->GetSignal("meas_gps_ang")->SetResult(msg.measured_wz);
  else message->GetSignal("meas_gps_ang")->SetResult(0);

  if (currTime - msg.timesec_rtk < 2) message->GetSignal("rtk_ack")->SetResult(msg.rtk_status);
  else message->GetSignal("rtk_ack")->SetResult(0);

  // Getting logging status using param value set by data logger
  bool has_log_status = node_->get_parameter("/log_status", log_st);
  if (has_log_status) {
    // If param has value then get it otherwise send `0`
    //log_st = this->get_parameter("/log_status").as_int()
    message->GetSignal("log_ack")->SetResult(log_st);
  }
  else message->GetSignal("log_ack")->SetResult(0);

  frame_ = message->GetFrame();
  this->pub_can_->publish(frame_);
}

// Publish AU status at 50hz
void DeepOrangeDbwCan::publishAuStatus() { //const ros::TimerEvent& event) {
  auStatusMsg_.header.stamp = builtin_interfaces::msg::Time(); //rclcpp::Time::now();

  // Updating values
  auStatusMsg_.rtk_status = *rtk_status_ptr_;
  auStatusMsg_.measured_vx = *measVx_ptr_;
  auStatusMsg_.measured_wz = *measWz_ptr_;

  // Updating time stamp
  auStatusMsg_.timesec_vx = *time_Vx_ptr_;
  auStatusMsg_.timesec_wz = *time_Wz_ptr_;
  auStatusMsg_.timesec_rtk = *time_Rtk_ptr_;

  // Publishing values
  this->pub_auStatus_->publish(auStatusMsg_);
}

// Getting RTK status
void DeepOrangeDbwCan::getRtkStatus(const novatel_oem7_msgs::msg::INSPVAX& msg) {
  *time_Rtk_ptr_ = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9;
  if (msg.pos_type.type == 50 || msg.pos_type.type == 56)   {
    *rtk_status_ptr_ = 1;
  }
  else {
    *rtk_status_ptr_ = 0;
  }
}

// Getting Measured Wz
void DeepOrangeDbwCan::getMeasuredWz(const sensor_msgs::msg::Imu& msg) {
  *time_Wz_ptr_ = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9;
  vectorWz_.push_back(msg.angular_velocity.z);

  if (vectorWz_.size() == 4) {
    for (int i = 0; i < 4; i++) {
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
void DeepOrangeDbwCan::getMeasuredVx(const nav_msgs::msg::Odometry& msg) {
  *time_Vx_ptr_ = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9;
  vectorVx_.push_back(msg.twist.twist.linear.x);

  if (vectorVx_.size() == 2) {
    for (int i = 0; i < 2; i++) {
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

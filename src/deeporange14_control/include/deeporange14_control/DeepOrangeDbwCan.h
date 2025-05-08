/* 
A class to manage can msgs (parse and publish to can_tx/rx) using dbc file provided
Relies on DbwSupervisor to figure out what to send to Raptor. Uses "ros_state" topic to get this info
Receives CAN data from socketcan node and provides info to DbwSupervisor
*/

#ifndef DEEPORANGE14_CONTROL__DEEPORANGEDBWCAN_H_
#define DEEPORANGE14_CONTROL__DEEPORANGEDBWCAN_H_

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

// TODO: are all of these needed?
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <novatel_oem7_msgs/msg/inspvax.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <can_dbc_parser/DbcMessage.hpp>
#include <can_dbc_parser/DbcSignal.hpp>
#include <can_dbc_parser/Dbc.hpp>
#include <can_dbc_parser/DbcBuilder.hpp>

#include <deeporange14_control/dispatch_can_msgs.h>
#include <deeporange14_msgs/msg/au_status.hpp>
#include <deeporange14_msgs/msg/mobility.hpp>
#include <deeporange14_msgs/msg/raptor_state.hpp>

namespace deeporange14 {
class DeepOrangeDbwCan {
 public:
  DeepOrangeDbwCan(rclcpp::Node::SharedPtr node);
  ~DeepOrangeDbwCan();

 private:
  void recvCAN(const can_msgs::msg::Frame::SharedPtr msg);
  // void publishCAN(const ros::TimerEvent& event);
  void publishCommandstoCAN(const deeporange14_msgs::msg::Mobility& msg);
  void publishAuStatustoCAN(const deeporange14_msgs::msg::AuStatus& msg);
  void getMeasuredVx(const nav_msgs::msg::Odometry& msg);
  void getMeasuredWz(const sensor_msgs::msg::Imu& msg);
  void getRtkStatus(const novatel_oem7_msgs::msg::INSPVAX& msg);
  void publishAuStatus();  // const ros::TimerEvent& event);
  void readDbcFile(std::string &dbc_file, std::string &dbc_raw);

  rclcpp::Node::SharedPtr node_;

  // ROS timer object
  rclcpp::TimerBase::SharedPtr timer_;

  // Publishers
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_can_;
  rclcpp::Publisher<deeporange14_msgs::msg::RaptorState>::SharedPtr pub_raptorState_;
  rclcpp::Publisher<deeporange14_msgs::msg::AuStatus>::SharedPtr pub_auStatus_;

  // Subscribers
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
  rclcpp::Subscription<deeporange14_msgs::msg::Mobility>::SharedPtr sub_auMobility_;
  // rclcpp::Subscriber<deeporange14_msgs::msg::AuStatus>::SharedPtr sub_auStatus_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_gpsImu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVAX>::SharedPtr sub_rtk_;
  rclcpp::Subscription<deeporange14_msgs::msg::AuStatus>::SharedPtr sub_autonomyLog_;

  // Published msgs
  deeporange14_msgs::msg::RaptorState raptorMsg_;
  deeporange14_msgs::msg::AuStatus auStatusMsg_;

  // Subscribed msgs
  deeporange14_msgs::msg::Mobility mobilityMsg_;
  nav_msgs::msg::Odometry odometryMsg_;
  sensor_msgs::msg::Imu gpsImuMsg_;

  // Variables for measured velocities
  std::vector<float> vectorWz_;
  float averageWz_ = 0;
  std::vector<float> vectorVx_;
  float averageVx_ = 0;

  // Frame ID
  std::string frameId_;
  can_msgs::msg::Frame frame_;

  // dbc files
  NewEagle::Dbc rosDbc_;
  NewEagle::Dbc raptorDbc_;
  std::string dbcFileRos_;
  std::string dbcFileRaptor_;
  std::string dbcRosRaw_;
  std::string dbcRaptorRaw_;

  // Heartbeat var & ptr
  int ros_hb_ = 0;
  int *ros_hb_ptr_ = &ros_hb_;

  // Brake pressure var
  int brk_ip_Rpres_;  // right
  int brk_ip_Lpres_;  // left

  // Variables to store rtk, log, and gps velocities and their pointers
  int rtk_status_ = 0;
  double meas_Vx_ = 0.0;
  double meas_Wz_ = 0.0;
  int *rtk_status_ptr_ = &rtk_status_;
  double *measVx_ptr_ = &meas_Vx_;
  double *measWz_ptr_ = &meas_Wz_;

  // Variables to store timestamp and their pointers
  int time_Vx_ = 0;
  int time_Wz_ = 0;
  int time_Rtk_ = 0;
  int *time_Vx_ptr_ = &time_Vx_;
  int *time_Wz_ptr_ = &time_Wz_;
  int *time_Rtk_ptr_ = &time_Rtk_;

  // Param to retrive
  int log_st;
  std::string topic_ns = "/deeporange1314";
};
}  // namespace deeporange14

#endif  // DEEPORANGE14_CONTROL__DEEPORANGEDBWCAN_H_

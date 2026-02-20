/* A high level state machine to interact with Raptor to control brakes torque command */

#ifndef DEEPORANGE_STATE_SUPERVISOR_H_
#define DEEPORANGE_STATE_SUPERVISOR_H_

#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <deeporange14_control/DeepOrangeStateEnums.h>
#include <deeporange14_msgs/AutonomyCommandMsg.h>
#include <deeporange14_msgs/AutonomyMeasurementMsg.h>

namespace deeporange14 {
class DeepOrangeStateSupervisor {
 public:
  DeepOrangeStateSupervisor(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
  ~DeepOrangeStateSupervisor();

 private:
  void pubMeasurements(const ros::TimerEvent& event);
  void pubCommands();

  void getCmdVel(const geometry_msgs::Twist::ConstPtr& msg);
  void getStopRos(const std_msgs::Bool::ConstPtr& msg);
  void getMissionStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
  void getMeasurements(const deeporange14_msgs::AutonomyMeasurementMsg::ConstPtr& msg);

  void updateControlCommands(const ros::TimerEvent& event);
  void updateMissionStatusBools();
  void updateStateMachine();

  // member variables
  float vx_meas_;
  float vx_cmd_;
  float curv_meas_;
  float curv_cmd_;
  float wx_meas_calc_;

  float last_cmd_recv_time_;
  float ros_stop_time_;

  bool stack_fault_;

  uint dbw_state_;
  uint au_state_;
  uint prev_au_state_;

  uint8_t mission_status_;

  bool stop_ros_;
  bool mission_running_;
  bool mission_completed_;
  bool mission_aborted_;

  // Publishers
  ros::Publisher pub_au_cmd_;
  ros::Publisher pub_vx_meas_;
  ros::Publisher pub_curv_meas_;
  ros::Publisher pub_wx_meas_;

  // Subscribers
  ros::Subscriber sub_cmd_vel_;
  ros::Subscriber sub_stop_ros_;
  ros::Subscriber sub_mission_status_;
  ros::Subscriber sub_au_meas_;

  ros::Timer timer_;
  ros::Timer meas_timer_;

  // Init the msg variables
  deeporange14_msgs::AutonomyCommandMsg au_cmd_msg_;
  actionlib_msgs::GoalStatus goal_status_dummy_;

  // variables for parameters
  // TODO - make this a parameter
  std::string topic_ns_ = "/deeporange1314";
  std::string topic_au_cmd_;
  std::string topic_au_meas_;
  int update_freq_hz_;
  float cmd_recv_timeout_s_;

};
}  // namespace deeporange14

#endif  // DEEPORANGE_STATE_SUPERVISOR_H_

/* A high level state machine to interact with Raptor to control brakes torque command */

#ifndef DEEPORANGE_STATE_SUPERVISOR_H_
#define DEEPORANGE_STATE_SUPERVISOR_H_

#include <algorithm>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>  // TODO - not needed if mission_status is not sub'd

// TODO - are these needed?
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>

#include <deeporange14_control/DeepOrangeStateEnums.h>
#include <deeporange14_msgs/AutonomyCommandMsg.h>
#include <deeporange14_msgs/AutonomyMeasurementMsg.h>

namespace deeporange14 {
class DeepOrangeStateSupervisor {
 public:
  DeepOrangeStateSupervisor(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
  ~DeepOrangeStateSupervisor();

 private:
  void checkStackStatus(const geometry_msgs::Twist::ConstPtr& cmdVelMsg);
  void getMissionStatus(const std_msgs::String::ConstPtr& missionStatus);
  void getStopRos(const std_msgs::Bool::ConstPtr& stopRosMsg);

  void supervisorControlUpdate(const ros::TimerEvent& event);
  void getPhxStatus(const actionlib_msgs::GoalStatusArray::ConstPtr& statusMsg);

  void getMeasurements(const deeporange14_msgs::AutonomyMeasurementMsg::ConstPtr& msg);

  void updateROSState();

  // member variables
  bool raptor_hb_detected;
  bool stack_fault;
  bool dbw_ros_mode;
  std::string mission_status;
  bool stop_ros;

  allStates state;
  double raptor_hb_timestamp;
  double cmdvel_timestamp;
  double stop_ros_timestamp;
  double mission_update_timestamp;
  uint speed_state;
  uint8_t mppi_status;
  double counter;
  float cmdvel_timeout;
  float raptorhb_timeout;
  int update_freq;
  float brake_disengaged_threshold;

  // temp -- collecting what are used
  float vx_meas_;
  float vx_cmd_;
  float curv_meas_;
  float curv_cmd_;
  uint dbw_state_;
  uint au_state_;
  uint prev_au_state_;

  int fault_delay_s_;
  int fault_delay_ct_;
  int fault_delay_;

  // Publishers
  ros::Publisher pub_au_cmd_;

  // Subscribers
  ros::Subscriber sub_cmdVel_;
  ros::Subscriber sub_missionStatus_;
  ros::Subscriber sub_rosStop_;
  ros::Subscriber sub_stopRos_;
  ros::Subscriber sub_mppi_mission_;
  std::string topic_ns_ = "/deeporange1314";

  ros::Subscriber sub_au_meas_;

  ros::Timer timer_;

  // Init the msg variables
  deeporange14_msgs::AutonomyCommandMsg auCmdMsg_;
  deeporange14_msgs::AutonomyMeasurementMsg auMeasMsg_;
};
}  // namespace deeporange14

#endif  // DEEPORANGE_STATE_SUPERVISOR_H_

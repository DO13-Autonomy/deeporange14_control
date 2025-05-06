/* A high level state machine to interact with Raptor to control brakes torque command */

#ifndef DEEPORANGE_STATE_SUPERVISOR_H_
#define DEEPORANGE_STATE_SUPERVISOR_H_

#include <string>

#include <rclcpp/rclcpp.hpp>
//#include <ros/console.h>

// TODO: are all of these needed?
#include <actionlib_msgs/msg/goal_status_array.hpp>
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <deeporange14_control/DeeporangeStateEnums.h>
#include <deeporange14_msgs/msg/mission_status.hpp>
#include <deeporange14_msgs/msg/mobility.hpp>
#include <deeporange14_msgs/msg/raptor_state.hpp>
#include <deeporange14_msgs/msg/torque_cmd_stamped.hpp>

namespace deeporange14 {
class DeepOrangeStateSupervisor {
 public:
  DeepOrangeStateSupervisor(rclcpp::Node::SharedPtr node);
  ~DeepOrangeStateSupervisor();

 private:
  void checkStackStatus(const geometry_msgs::msg::Twist& cmdVelMsg);

  void getMissionStatus(const std_msgs::msg::String& missionStatus);
  void getTorqueValues(const deeporange14_msgs::msg::TorqueCmdStamped& controllerTrqValues);
  void getStopRos(const std_msgs::msg::Bool& stopRosMsg);
  void getRaptorMsg(const deeporange14_msgs::msg::RaptorState& raptorMsg);

  void supervisorControlUpdate(); //const ros::TimerEvent& event);
  void updateROSState();
  void getPhxStatus(const actionlib_msgs::msg::GoalStatusArray& statusMsg);

  rclcpp::Node::SharedPtr node_;

  // member variables
  bool raptor_hb_detected;
  bool stack_fault;
  bool dbw_ros_mode;
  std::string mission_status;
  float brkL_pr;
  float brkR_pr;
  float tqL_cmd_controller;
  float tqR_cmd_controller;
  bool stop_ros;
  bool raptorbrakeAck;
  int delay;
  int desired_delay;
  int delay_threshold;
  int prevSt;

  allStates state;
  double raptor_hb_timestamp;
  double cmdvel_timestamp;
  double stop_ros_timestamp;
  double mission_update_timestamp;
  uint speed_state;
  uint au_state;
  uint8_t mppi_status;
  double counter;
  float cmdvel_timeout;
  float raptorhb_timeout;
  int update_freq;
  float brake_disengaged_threshold;

  // ROS timer object
  rclcpp::TimerBase::SharedPtr timer;

  // Publishers
  rclcpp::Publisher<deeporange14_msgs::msg::Mobility>::SharedPtr pub_mobility;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_states;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmdVel;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_missionStatus;
  //rclcpp::Subscriber sub_brakeStatus;
  rclcpp::Subscription<deeporange14_msgs::msg::TorqueCmdStamped>::SharedPtr sub_rosController;
  //rclcpp::Subscriber sub_rosStop;
  rclcpp::Subscription<deeporange14_msgs::msg::RaptorState>::SharedPtr sub_raptorState;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_stopRos;
  rclcpp::Subscription<actionlib_msgs::msg::GoalStatusArray>::SharedPtr sub_mppi_mission;
  std::string topic_ns = "/deeporange1314";

  // Init the msg variables
  std_msgs::msg::UInt16 auStateMsg;
  deeporange14_msgs::msg::Mobility mobilityMsg;
  deeporange14_msgs::msg::TorqueCmdStamped trqvalues;
  deeporange14_msgs::msg::RaptorState raptorMsg;
};
}  // namespace deeporange14

#endif  // DEEPORANGE_STATE_SUPERVISOR_H_

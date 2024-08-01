#ifndef DEEPORANGE_VELOCITY_CONTROLLER_H_
#define DEEPORANGE_VELOCITY_CONTROLLER_H_

#include <math.h>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

#include <deeporange14_control/DeeporangeStateEnums.h>
#include <deeporange14_msgs/MobilityMsg.h>
#include <deeporange14_msgs/PIDComponentsMsg.h>
#include <deeporange14_msgs/TorqueCmdStamped.h>

namespace deeporange14
{
class VelocityController
{
 public:
  VelocityController(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
  ~VelocityController();

  // namespace string
  std::string topic_ns = "/deeporange1314";

  // publisher and subscriber
  ros::Subscriber sub_cmd_vel_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_brakes_;
  ros::Subscriber sub_moboility_msg_;
  ros::Publisher pub_cmd_trq_;
  ros::Publisher pub_cmd_vel_reprojected_;
  ros::Publisher pub_remap_state_;
  ros::Publisher pub_pid_components_;

  ros::Timer timer_;
  std_msgs::UInt8 remapping_state_msg_;

  // callback functions
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void publishTorques(const ros::TimerEvent& event);
  void brakeCallback(const std_msgs::Bool::ConstPtr& msg);
  void cmdMobilityCallback(const deeporange14_msgs::MobilityMsg::ConstPtr& msg);

  // velocity reprojection to the admissible range
  void linearVelocityReprojection(double &v, double &w);
  void twistReprojection(double &v, double &w);

  // rate limiter on the commands
  void rateLimiter(double &prev_u_, double &u_);

  // member variables -- velocities (commanded and odom)
  double cmdLinX_;
  double cmdAngZ_;
  double vehLinX_;
  double vehAngZ_;
  double cmd_turn_curvature_;
  // double odom_turn_curvature_;

  // feedforward terms
  double x0_;
  double x1_;
  double a_;
  double b_;

  // member variables -- feedforward and PID torques
  double tqDiff_ff_;
  double tqCom_ff_;
  double tqDiff_PID_;
  double tqComm_PID_;
  double tqDiff_;
  double tqComm_;
  double tqL_;
  double tqR_;

  // member variables -- PID controller gains and errors
  double errLinX_current_;
  double errLinX_prev_;
  double errLinX_integral_;
  double errLinX_derivative_;
  double errOmega_current_;
  double errOmega_prev_;
  double errOmega_integral_;
  double errOmega_derivative_;
  double kP_linX_;
  double kI_linX_;
  double kD_linX_;
  double kP_omega_;
  double kI_omega_;
  double kD_omega_;
  bool brake_engage_;

  // member variables -- anti-windup values
  double tq_Max_;
  double tq_Min_;

  // double curvature_rate_limit_;
  // double trackwidth;
  double v_sz;                            // intersection of max curvature line and max lateral acceleration curve
  double R_min;                           // minimum allowable radius of curvature
  double lat_acc_max;                     // maximum allowable lateral acceleration
  double prev_time_;
  double current_time_;
  double dt_;
  double prev_v_;
  double prev_omega_;
  double deadband_velocity;
  double v_moving_ss;
  double v_moving;
  double v_stopped;

  // member variables -- velocity limits
  double min_velocity;  // min positive linear velocity
  double max_velocity;  // max positive linear velocity
  double min_omega;     // min positive angular velocity
  double max_omega;     // max positive linear velocity

  // state-enumeration
  uint8_t autonomy_state_;
  allStates remapping_state;

  // Rate limiter constants
  double dec_min;
  double a_acc;
  double b_acc;
  double a_dec;
  double b_dec;
  double acc_max;
  double dec_max;
  double rmin;
  double rmax;
  double smoothing_factor;
};
}  // namespace deeporange14

#endif  // DEEPORANGE_VELOCITY_CONTROLLER_H_

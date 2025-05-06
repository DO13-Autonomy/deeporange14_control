#ifndef DEEPORANGE_VELOCITY_CONTROLLER_H_
#define DEEPORANGE_VELOCITY_CONTROLLER_H_

#include <algorithm>
#include <limits>
#include <math.h>
#include <string>

#include <rclcpp/rclcpp.hpp>
//#include <ros/console.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include <deeporange14_control/DeeporangeStateEnums.h>
#include <deeporange14_msgs/msg/cmd_vel_cntrl.hpp>
#include <deeporange14_msgs/msg/mobility.hpp>
#include <deeporange14_msgs/msg/pid_components.hpp>
#include <deeporange14_msgs/msg/torque_cmd_stamped.hpp>

namespace deeporange14 {
class VelocityController {
 public:
  VelocityController(rclcpp::Node::SharedPtr node);
  ~VelocityController();

  // callback functions
  void cmdVelCallback(const geometry_msgs::msg::Twist& msg);
  void odomCallback(const nav_msgs::msg::Odometry& msg);
  void publishTorques();
  void brakeCallback(const std_msgs::msg::Bool& msg);
  void cmdMobilityCallback(const deeporange14_msgs::msg::Mobility& msg);

  // velocity reprojection to the admissible range
  void linearVelocityReprojection(double &v, double &w);
  void twistReprojection(double &v, double &w);

  // rate limiter on the commands
  void rateLimiter_LinX(double &prev_u_, double &u_);
  void rateLimiter_AngZ(double &prev_omega, double &omega_);

  void ReadParameters();

  // namespace string
  std::string topic_ns = "/deeporange1314";

  rclcpp::Node::SharedPtr node_;

  // publisher and subscriber
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_brakes_;
  rclcpp::Subscription<deeporange14_msgs::msg::Mobility>::SharedPtr sub_moboility_msg_;

  rclcpp::Publisher<deeporange14_msgs::msg::TorqueCmdStamped>::SharedPtr pub_cmd_trq_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_reprojected_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_remap_state_;
  rclcpp::Publisher<deeporange14_msgs::msg::PIDComponents>::SharedPtr pub_pid_components_;
  rclcpp::Publisher<deeporange14_msgs::msg::CmdVelCntrl>::SharedPtr pub_cmd_vel_cntrl;

  rclcpp::TimerBase::SharedPtr timer_;
  std_msgs::msg::UInt16 remapping_state_msg_;

  // member variables -- velocities (commanded and odom)
  double cmdLinX_;
  double cmdAngZ_;
  double vehLinX_;
  double vehAngZ_;
  double cmd_turn_curvature_;

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

  // rate limiter constants for linear velocity
  double dec_min_v;
  double a_acc_v;
  double b_acc_v;
  double a_dec_v;
  double b_dec_v;
  double acc_max_v;
  double dec_max_v;
  double rmin_v;
  double rmax_v;
  double smoothing_factor_v;

  // rate limiter constants for angular velocity
  double dec_min_w;
  double a_acc_w;
  double b_acc_w;
  double a_dec_w;
  double b_dec_w;
  double acc_max_w;
  double dec_max_w;
  double rmin_w;
  double rmax_w;
  double smoothing_factor_w;
};
}  // namespace deeporange14

#endif  // DEEPORANGE_VELOCITY_CONTROLLER_H_

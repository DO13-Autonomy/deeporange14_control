#include <deeporange14_control/DeepOrangeVelocityController.h>

namespace deeporange14
{
VelocityController::VelocityController(rclcpp::Node::SharedPtr node)
: node_(node)
{
  // initialize other member variables and set up subscriptions and publishers
  sub_cmd_vel_ = node->create_subscription<geometry_msgs::msg::Twist>(
    std::string(topic_ns + "/cmd_vel"),
    10,
    std::bind(&VelocityController::cmdVelCallback, this, std::placeholders::_1));
  sub_odom_ = node->create_subscription<nav_msgs::msg::Odometry>(
    std::string(topic_ns + "/odom"),
    10,
    std::bind(&VelocityController::odomCallback, this, std::placeholders::_1));
  sub_moboility_msg_ = node->create_subscription<deeporange14_msgs::msg::Mobility>(
    std::string(topic_ns + "/cmd_mobility"),
    10,
    std::bind(&VelocityController::cmdMobilityCallback, this, std::placeholders::_1));
  sub_brakes_ = node->create_subscription<std_msgs::msg::Bool>(
    std::string(topic_ns + "/brake_command"),
    10,
    std::bind(&VelocityController::brakeCallback, this, std::placeholders::_1));

  pub_cmd_trq_ = node->create_publisher<deeporange14_msgs::msg::TorqueCmdStamped>(
    std::string(topic_ns + "/cmd_trq"),
    10);
  pub_cmd_vel_reprojected_ = node->create_publisher<geometry_msgs::msg::Twist>(
    std::string(topic_ns + "/cmd_vel_reprojected"),
    10);
  pub_remap_state_ = node->create_publisher<std_msgs::msg::UInt16>(
    std::string(topic_ns + "/remapping_state"),
    10);
  pub_pid_components_ = node->create_publisher<deeporange14_msgs::msg::PIDComponents>(
    std::string(topic_ns + "/pid_components"),
    10);
  pub_cmd_vel_cntrl = node->create_publisher<deeporange14_msgs::msg::CmdVelCntrl>(
    std::string(topic_ns + "/cmd_vel_cntrl"),
    10);

  // declare parameters with default values
  node->declare_parameter("pid_gains.kP_linX", 150.0);
  node->declare_parameter("pid_gains.kI_linX", 30.0);
  node->declare_parameter("pid_gains.kD_linX", 0.0);
  node->declare_parameter("pid_gains.kP_omega", 200.0);
  node->declare_parameter("pid_gains.kI_omega", 8.0);
  node->declare_parameter("pid_gains.kD_omega", 0.0);

  node->declare_parameter("feedforward.x0", 1500.0);
  node->declare_parameter("feedforward.x1", 7.8);
  node->declare_parameter("feedforward.a", 4.37);
  node->declare_parameter("feedforward.b", 18.0);

  node->declare_parameter("torque_limits.tq_Max", 280.0);
  node->declare_parameter("torque_limits.tq_Min", -280.0);

  node->declare_parameter("velocity.max_velocity", 1.6);
  node->declare_parameter("velocity.min_velocity", -1.0);

  node->declare_parameter("omega_limits.max_omega", 0.6);
  node->declare_parameter("omega_limits.min_omega", 0.5);
  node->declare_parameter("omega_limits.R_min", 2.0);

  node->declare_parameter("deadband_velocity", 0.1);

  node->declare_parameter("rate_limiter.dec_min_for_lin_vel", -0.1);
  node->declare_parameter("rate_limiter.a_acc_for_lin_vel", 0.5);
  node->declare_parameter("rate_limiter.b_acc_for_lin_vel", 3.0);
  node->declare_parameter("rate_limiter.a_dec_for_lin_vel", -0.5);
  node->declare_parameter("rate_limiter.b_dec_for_lin_vel", -15.0);
  node->declare_parameter("rate_limiter.acc_max_for_lin_vel", 1.0);
  node->declare_parameter("rate_limiter.dec_max_for_lin_vel", -1);
  node->declare_parameter("rate_limiter.smoothing_factor_for_lin_vel", 20.0);

  node->declare_parameter("rate_limiter.dec_min_for_ang_vel", -0.1);
  node->declare_parameter("rate_limiter.a_acc_for_ang_vel", 0.5);
  node->declare_parameter("rate_limiter.b_acc_for_ang_vel", 3.0);
  node->declare_parameter("rate_limiter.a_dec_for_ang_vel", -0.5);
  node->declare_parameter("rate_limiter.b_dec_for_ang_vel", -15.0);
  node->declare_parameter("rate_limiter.acc_max_for_ang_vel", 1.0);
  node->declare_parameter("rate_limiter.dec_max_for_ang_vel", -1.0);
  node->declare_parameter("rate_limiter.smoothing_factor_for_ang_vel", 20.0);

  node->declare_parameter("dt", 0.02);

  node->declare_parameter("moving_velocity.v_moving_ss", 0.5);
  node->declare_parameter("moving_velocity.v_moving", 0.4);
  node->declare_parameter("moving_velocity.v_stopped", 0.01);

  VelocityController::ReadParameters();

  // other member variable initializations
  cmdLinX_ = 0.0;
  cmdAngZ_ = 0.0;
  vehLinX_ = 0.0;
  vehAngZ_ = 0.0;
  tqDiff_ff_ = 0.0;
  tqCom_ff_ = 0.0;
  tqDiff_PID_ = 0.0;
  tqComm_PID_ = 0.0;
  tqDiff_ = 0.0;
  tqComm_ = 0.0;
  tqL_ = 0.0;
  tqR_ = 0.0;
  errLinX_current_ = 0.0;
  errLinX_prev_ = 0.0;
  errLinX_integral_ = 0.0;
  errLinX_derivative_ = 0.0;
  errOmega_current_ = 0.0;
  errOmega_prev_ = 0.0;
  errOmega_integral_ = 0.0;
  errOmega_derivative_ = 0.0;
  cmd_turn_curvature_ = 0.0;
  prev_v_ = 0.0;
  prev_omega_ = 0.0;
  v_sz = max_omega * R_min;
  lat_acc_max = max_omega * v_sz;
  autonomy_state_ = AU_1_STARTUP;
  remapping_state = VEHICLE_STOPPED;

  float timer_period = 1.0 / 50;
  timer_ = node->create_wall_timer(
    std::chrono::duration<double, std::nano>(timer_period),
    std::bind(&VelocityController::publishTorques, this));
}

VelocityController::~VelocityController() {}

// function to handle parameter retrieval
void VelocityController::ReadParameters()
{
  node_->get_parameter("pid_gains.kP_linX", kP_linX_);
  node_->get_parameter("pid_gains.kI_linX", kI_linX_);
  node_->get_parameter("pid_gains.kD_linX", kD_linX_);
  node_->get_parameter("pid_gains.kP_omega", kP_omega_);
  node_->get_parameter("pid_gains.kI_omega", kI_omega_);
  node_->get_parameter("pid_gains.kD_omega", kD_omega_);

  node_->get_parameter("feedforward.x0", x0_);
  node_->get_parameter("feedforward.x1", x1_);
  node_->get_parameter("feedforward.a", a_);
  node_->get_parameter("feedforward.b", b_);

  node_->get_parameter("torque_limits.tq_Max", tq_Max_);
  node_->get_parameter("torque_limits.tq_Min", tq_Min_);

  node_->get_parameter("velocity.max_velocity", max_velocity);
  node_->get_parameter("velocity.min_velocity", min_velocity);

  node_->get_parameter("omega_limits.max_omega", max_omega);
  node_->get_parameter("omega_limits.min_omega", min_omega);
  node_->get_parameter("omega_limits.R_min", R_min);

  node_->get_parameter("deadband_velocity", deadband_velocity);

  node_->get_parameter("rate_limiter.dec_min_for_lin_vel", dec_min_v);
  node_->get_parameter("rate_limiter.a_acc_for_lin_vel", a_acc_v);
  node_->get_parameter("rate_limiter.b_acc_for_lin_vel", b_acc_v);
  node_->get_parameter("rate_limiter.a_dec_for_lin_vel", a_dec_v);
  node_->get_parameter("rate_limiter.b_dec_for_lin_vel", b_dec_v);
  node_->get_parameter("rate_limiter.acc_max_for_lin_vel", acc_max_v);
  node_->get_parameter("rate_limiter.dec_max_for_lin_vel", dec_max_v);
  node_->get_parameter("rate_limiter.smoothing_factor_for_lin_vel", smoothing_factor_v);

  node_->get_parameter("rate_limiter.dec_min_for_ang_vel", dec_min_w);
  node_->get_parameter("rate_limiter.a_acc_for_ang_vel", a_acc_w);
  node_->get_parameter("rate_limiter.b_acc_for_ang_vel", b_acc_w);
  node_->get_parameter("rate_limiter.a_dec_for_ang_vel", a_dec_w);
  node_->get_parameter("rate_limiter.b_dec_for_ang_vel", b_dec_w);
  node_->get_parameter("rate_limiter.acc_max_for_ang_vel", acc_max_w);
  node_->get_parameter("rate_limiter.dec_max_for_ang_vel", dec_max_w);
  node_->get_parameter("rate_limiter.smoothing_factor_for_ang_vel", smoothing_factor_w);

  node_->get_parameter("dt", dt_);

  node_->get_parameter("moving_velocity.v_moving_ss", v_moving_ss);
  node_->get_parameter("moving_velocity.v_moving", v_moving);
  node_->get_parameter("moving_velocity.v_stopped", v_stopped);
}


// define the odom callback -- to be used by the controller as a feedback of the actual vehicle velocity
void VelocityController::odomCallback(const nav_msgs::msg::Odometry & msg)
{
  vehLinX_ = msg.twist.twist.linear.x;
  vehAngZ_ = msg.twist.twist.angular.z;
}

// define the brake callback -- used to determine when the the controller needs to be used to control the vehicle speeds
void VelocityController::brakeCallback(const std_msgs::msg::Bool & msg)
{
  brake_engage_ = msg.data;
}

// calback for reading the current state of the autonomy state-machine
void VelocityController::cmdMobilityCallback(const deeporange14_msgs::msg::Mobility & msg)
{
  autonomy_state_ = msg.au_state;
}

void VelocityController::cmdVelCallback(const geometry_msgs::msg::Twist & msg)
{
  cmdLinX_ = msg.linear.x;
  cmdAngZ_ = msg.angular.z;  // will be positive for nlopt, negative for mppi
  errLinX_prev_ = errLinX_current_;
  errOmega_prev_ = errOmega_current_;

  // integrator reset on transition to 'waiting for execution' state
  // after a mission is completed or cancelled, move back to 'startup' state and transition to 'wait execution' state
  if (autonomy_state_ == AU_3_ROS_MODE_EN) {
    errLinX_integral_ = 0.0;
    errOmega_integral_ = 0.0;
    tqL_ = 0.0;
    tqR_ = 0.0;
    prev_v_ = 0.0;
    prev_omega_ = 0.0;
    remapping_state = VEHICLE_STOPPED;
    RCLCPP_WARN(node_->get_logger(), "Left: %f, Right: %f", tqL_, tqR_);
    // RCLCPP_INFO(node_->get_logger(), "Velocity error integral: %f, Curvature error integral: %f",
    //  errLinX_integral_, errOmega_integral_);
  } else if (autonomy_state_ == AU_5_ROS_CONTROLLED || autonomy_state_ == AU_4_DISENGAGING_BRAKES) {
    // letting the controller kick in only when we move in the appropriate autonomy state
    // rate limiting the linear velocity and the curvature
    // velocity reprojection on the commanded velocities
    double reprojection_x = cmdLinX_;
    double reprojection_w = cmdAngZ_;

    RCLCPP_INFO(node_->get_logger(), "Angular Velocity from to rate limiter is : %f", cmdAngZ_);
    this->rateLimiter_LinX(prev_v_, cmdLinX_);
    this->rateLimiter_AngZ(prev_omega_, cmdAngZ_);
    RCLCPP_INFO(node_->get_logger(), "Angular Velocity From Rate Limter is : %f", cmdAngZ_);

    double ratelimiter_x = cmdLinX_;
    double ratelimiter_w = cmdAngZ_;

    deeporange14_msgs::msg::CmdVelCntrl cntrlmsg;
    cntrlmsg.reprojection_x = reprojection_x;
    cntrlmsg.reprojection_w = reprojection_w;
    cntrlmsg.ratelimiter_x = ratelimiter_x;
    cntrlmsg.ratelimiter_w = ratelimiter_w;

    this->pub_cmd_vel_cntrl->publish(cntrlmsg);

    cmd_turn_curvature_ = (cmdLinX_ != 0.0 && cmdAngZ_ != 0.0) ? (cmdAngZ_ / cmdLinX_) : 0.0;

    // publishing these results on a different topic to compare the changes
    geometry_msgs::msg::Twist cmd_vel_reprojected_;
    cmd_vel_reprojected_.linear.x = cmdLinX_;
    cmd_vel_reprojected_.angular.z = cmdAngZ_;
    this->pub_cmd_vel_reprojected_->publish(cmd_vel_reprojected_);

    tqDiff_ff_ = (x0_ * cmd_turn_curvature_) / (1 + x1_ * std::abs(cmd_turn_curvature_));
    tqCom_ff_ = ((cmdLinX_ > 0) - (cmdLinX_ < 0)) * (a_ * std::abs(cmdLinX_) + b_);

    // current errors
    errLinX_current_ = cmdLinX_ - vehLinX_;
    errOmega_current_ = cmdAngZ_ - vehAngZ_;
    RCLCPP_INFO(node_->get_logger(), "Error in Angular velocities is %f", errOmega_current_);

    // current derivatives
    errLinX_derivative_ = (errLinX_current_ - errLinX_prev_) / dt_;
    errOmega_derivative_ = (errOmega_current_ - errOmega_prev_) / dt_;

    // discrete controller output
    tqComm_PID_ = (kP_linX_ * errLinX_current_) + (kI_linX_ * errLinX_integral_) +
      (kD_linX_ * errLinX_derivative_);
    tqDiff_PID_ = (kP_omega_ * errOmega_current_) + (kI_omega_ * errOmega_integral_) +
      (kD_omega_ * errOmega_derivative_);

    // publishing individual components of the PID on the PIDComponentsMsg
    deeporange14_msgs::msg::PIDComponents pid_components_msg_;
    pid_components_msg_.p_vx = kP_linX_ * errLinX_current_;
    pid_components_msg_.i_vx = kI_linX_ * errLinX_integral_;
    pid_components_msg_.d_vx = kD_linX_ * errLinX_derivative_;
    pid_components_msg_.p_wz = kP_omega_ * errOmega_current_;
    pid_components_msg_.i_wz = kI_omega_ * errOmega_integral_;
    pid_components_msg_.d_wz = kD_omega_ * errOmega_derivative_;
    pub_pid_components_->publish(pid_components_msg_);

    // printing the components
    RCLCPP_INFO(
      node_->get_logger(),
      "PVx: %f, IVx: %f, DVx: %f",
      kP_linX_ * errLinX_current_,
      kI_linX_ * errLinX_integral_,
      kD_linX_ * errLinX_derivative_);
    RCLCPP_INFO(
      node_->get_logger(),
      "PWz: %f, IWz: %f, DWz: %f",
      kP_omega_ * errOmega_current_,
      kI_omega_ * errOmega_integral_,
      kD_omega_ * errOmega_derivative_);

    // feedforward + PID output
    tqDiff_ = tqDiff_ff_ + tqDiff_PID_;
    tqComm_ = tqCom_ff_ + tqComm_PID_;
    // RCLCPP_WARN(node_->get_logger(), "ff: %f, pid: %f",tqCom_ff_,tqComm_PID_);

    // splitting the common and differential torque into left and right torque
    tqL_ = tqComm_ - tqDiff_;
    tqR_ = tqComm_ + tqDiff_;

    // anti-windup behavior
    if (((tqL_ >= tq_Max_ || tqR_ >= tq_Max_) || ((tqL_ <= tq_Min_) || (tqR_ <= tq_Min_)))) {
      RCLCPP_WARN(node_->get_logger(), "Saturated Left: %f, Saturated Right: %f", tqL_, tqR_);
      if (tqComm_PID_ * errLinX_current_ > 0) {
        // stop integration for common torque for the next timestep, hence only curvature integral updated
        errOmega_integral_ += errOmega_current_ * dt_;
      } else {
        // resume integration for common torque the next timestep, hence both integrals updated
        errLinX_integral_ += errLinX_current_ * dt_;
        errOmega_integral_ += errOmega_current_ * dt_;
      }
      // publish the torques at saturation limit in either case
      tqL_ = std::max((std::min(tqL_, tq_Max_)), tq_Min_);
      tqR_ = std::max((std::min(tqR_, tq_Max_)), tq_Min_);
    } else {
      RCLCPP_WARN(node_->get_logger(), "Unsaturated Left: %f, Unsaturated Right: %f", tqL_, tqR_);
      // only update the error integrals, and NOT limit the torques since we haven't reached the limit
      errLinX_integral_ += errLinX_current_ * dt_;
      errOmega_integral_ += errOmega_current_ * dt_;
    }
    // RCLCPP_INFO(node_->get_logger(), "Curvature error integral: %f",errOmega_integral_);
  } else {
    // publish zero torques for all other states since brakes are enabled and velocity controller shouldn't kick in
    tqL_ = 0.0;
    tqR_ = 0.0;
  }
}

void VelocityController::linearVelocityReprojection(double & v, double & w)
{
  switch (remapping_state) {
    case VEHICLE_STOPPED:
      remapping_state_msg_.data = 0;
      this->pub_remap_state_->publish(remapping_state_msg_);

      RCLCPP_INFO(node_->get_logger(), "VEHICLE STOPPED");

      if (std::abs(v) > 0 || std::abs(w) > 0) {
        // move into the accelerating state
        v = std::max(v, v_moving_ss);  // minimum steady state velocity that we want the vehicle to move forward with
        remapping_state = VEHICLE_ACCELERATING;
        break;
      } else {
        v = 0;
        // do nothing, remain in this state
        break;
      }

    case VEHICLE_ACCELERATING:
      remapping_state_msg_.data = 1;
      this->pub_remap_state_->publish(remapping_state_msg_);

      RCLCPP_INFO(node_->get_logger(), "VEHICLE ACCELERATING");

      if (std::abs(vehLinX_) >= v_moving) {
        remapping_state = VEHICLE_MOVING;
        break;
      } else {
        // do nothing, remain in this state
        v = std::max(v, v_moving_ss);
        break;
      }

    case VEHICLE_MOVING:
      remapping_state_msg_.data = 2;
      this->pub_remap_state_->publish(remapping_state_msg_);

      RCLCPP_INFO(node_->get_logger(), "VEHICLE MOVING");

      // v stays the same and we do not need to reproject
      if (std::abs(vehLinX_) <= v_stopped) {
        v = 0;
        remapping_state = VEHICLE_STOPPED;
        break;
      } else {
        // do nothing, remain in this state
        break;
      }
  }
}

void VelocityController::twistReprojection(double & v, double & w)
{
// Function to reproject commanded stack velocities onto a velocity
// space that is executable by the Deep Orange 14 vehicle

  // Calculating commanded radius of curvature and lateral acceleration
  double R = (w != 0) ? v / w : std::numeric_limits<double>::infinity();
  double lat_acc = v * w;

  // Applying linear velocity limits to bring it into correct zone
  if (v < min_velocity) {
    v = min_velocity;  // limited to max forward
  }

  if (v > max_velocity) {
    v = max_velocity;  // limited to max forward
  }

  if (fabs(v) <= deadband_velocity) {
    // First Zone: Deadband (If small linear velocities -> linear only and make angular velocities zero to avoid stall)
    w = 0;
  } else if (fabs(v) <= v_sz && fabs(R) < R_min) {
    // Second zone: Commanded curvature should not exceed max curvature
    w = v / R_min * (w / fabs(w));
    if (!isfinite(w)) {
      RCLCPP_WARN(node_->get_logger(), "Deep Orange: w is not finite within reprojection");
    }
  } else if (fabs(v) <= max_velocity && fabs(lat_acc) > lat_acc_max) {
    // Third zone: Commanded lateral acceleration should not exceed max
    // Maintaining same curvature but reducing v and w to lie on v*w = lat_acc_max
    v = sqrt(lat_acc_max * fabs(R)) * (v / fabs(v));
    if (!isfinite(v)) {
      RCLCPP_WARN(node_->get_logger(), "Deep Orange: v is not finite within reprojection");
    }

    w = sqrt(lat_acc_max / fabs(R)) * (w / fabs(w));
    if (!isfinite(w)) {
      RCLCPP_WARN(node_->get_logger(), "Deep Orange: w is not finite within reprojection");
    }
  }
// TODO: State machine for accelerating, decelerating, stopped
}

// rate limiter for linear velocity
void VelocityController::rateLimiter_LinX(double & prev_u_, double & u_)
{
  // if positive command
  if (u_ > 0) {
    if (prev_u_ >= 0) {
      rmax_v = std::min(a_acc_v + b_acc_v * std::abs(prev_u_), acc_max_v);
      rmin_v = std::max(a_dec_v + b_dec_v * std::abs(prev_u_), dec_max_v);
    } else {
      rmax_v = std::min(-(a_dec_v + b_dec_v * std::abs(prev_u_)), -dec_max_v);
      rmin_v = std::max(-(a_acc_v + b_acc_v * std::abs(prev_u_)), -acc_max_v);
    }
  } else if (u_ < 0) {
    if (prev_u_ < 0) {
      rmax_v = std::min(-(a_dec_v + b_dec_v * std::abs(prev_u_)), -dec_max_v);
      rmin_v = std::max(-(a_acc_v + b_acc_v * std::abs(prev_u_)), -acc_max_v);
    } else {
      rmax_v = std::min(a_acc_v + b_acc_v * std::abs(prev_u_), acc_max_v);
      rmin_v = std::max(a_dec_v + b_dec_v * std::abs(prev_u_), dec_max_v);
    }
  } else {
    if (prev_u_ > 0) {
      rmax_v = 0;
      rmin_v = std::max(
        -std::max(
          a_dec_v + b_dec_v * prev_u_, smoothing_factor_v * prev_u_) + dec_min_v, dec_max_v);
    } else {
      rmax_v = std::min(
        std::max(
          a_dec_v + b_dec_v * prev_u_, -smoothing_factor_v * prev_u_) - dec_min_v, -dec_max_v);
      rmin_v = 0;
    }
  }

  // linear velocity rate limiter
  double rate_u_ = (u_ - prev_u_) / dt_;
  double allowable_rate_u_ = std::max(std::min(rate_u_, rmax_v), rmin_v);

  u_ = prev_u_ + allowable_rate_u_ * dt_;
  prev_u_ = u_;
}

// rate limiter for angular velocity
void VelocityController::rateLimiter_AngZ(double & prev_w_, double & w_)
{
  // if positive command
  if (w_ > 0) {
    if (prev_w_ >= 0) {
      rmax_w = std::min(a_acc_w + b_acc_w * std::abs(prev_w_), acc_max_w);
      rmin_w = std::max(a_dec_w + b_dec_w * std::abs(prev_w_), dec_max_w);
    } else {
      rmax_w = std::min(-(a_dec_w + b_dec_w * std::abs(prev_w_)), -dec_max_w);
      rmin_w = std::max(-(a_acc_w + b_acc_w * std::abs(prev_w_)), -acc_max_w);
    }
  } else if (w_ < 0) {
    if (prev_w_ < 0) {
      rmax_w = std::min(-(a_dec_w + b_dec_w * std::abs(prev_w_)), -dec_max_w);
      rmin_w = std::max(-(a_acc_w + b_acc_w * std::abs(prev_w_)), -acc_max_w);
    } else {
      rmax_w = std::min(a_acc_w + b_acc_w * std::abs(prev_w_), acc_max_w);
      rmin_w = std::max(a_dec_w + b_dec_w * std::abs(prev_w_), dec_max_w);
    }
  } else {
    if (prev_w_ > 0) {
      rmax_w = 0;
      rmin_w = std::max(
        -std::max(
          a_dec_w + b_dec_w * prev_w_, smoothing_factor_w * prev_w_) + dec_min_w, dec_max_w);
    } else {
      rmax_w = std::min(
        std::max(
          a_dec_w + b_dec_w * prev_w_, -smoothing_factor_w * prev_w_) - dec_min_w, -dec_max_w);
      rmin_w = 0;
    }
  }

  // linear velocity rate limiter
  double rate_w_ = (w_ - prev_w_) / dt_;
  double allowable_rate_w_ = std::max(std::min(rate_w_, rmax_w), rmin_w);
  RCLCPP_INFO(
    node_->get_logger(),
    "CURRENT RATE: %f, Maximum Rate Limiter ANG: %f,Minimum Rate Limiter ANG: %f",
    rate_w_,
    rmax_w,
    rmin_w);

  w_ = prev_w_ + allowable_rate_w_ * dt_;
  prev_w_ = w_;
}

void VelocityController::publishTorques()
{
  deeporange14_msgs::msg::TorqueCmdStamped trq_cmd_;

  trq_cmd_.header.stamp = builtin_interfaces::msg::Time();
  trq_cmd_.tql_cmd = tqL_;
  trq_cmd_.tqr_cmd = tqR_;

  this->pub_cmd_trq_->publish(trq_cmd_);
}
}  // namespace deeporange14

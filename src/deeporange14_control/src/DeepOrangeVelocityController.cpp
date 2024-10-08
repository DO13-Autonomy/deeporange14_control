#include <deeporange14_control/DeepOrangeVelocityController.h>

namespace deeporange14{
    VelocityController::VelocityController(ros::NodeHandle &node, ros::NodeHandle &priv_nh){
        priv_nh.param("pid_gains/kP_linX", kP_linX_, 150.0);
        priv_nh.param("pid_gains/kI_linX", kI_linX_, 30.0);
        priv_nh.param("pid_gains/kD_linX", kD_linX_, 0.0);
        priv_nh.param("pid_gains/kP_omega", kP_omega_, 200.0);
        priv_nh.param("pid_gains/kI_omega", kI_omega_, 8.0);
        priv_nh.param("pid_gains/kD_omega", kD_omega_, 0.0);

        priv_nh.param("feedforward/x0", x0_, 1500.0);
        priv_nh.param("feedforward/x1", x1_, 7.8);
        priv_nh.param("feedforward/a", a_, 4.37);
        priv_nh.param("feedforward/b", b_, 18.0);

        priv_nh.param("torque_limits/tq_Max", tq_Max_, 280.0);
        priv_nh.param("torque_limits/tq_Min", tq_Min_, -280.0);

        priv_nh.param("velocity/max_velocity", max_velocity, 1.6);
        priv_nh.param("velocity/min_velocity", min_velocity, -1.0);

        priv_nh.param("omega_limits/max_omega", max_omega, 0.6);
        priv_nh.param("omega_limits/min_omega", min_omega, 0.5);
        priv_nh.param("omega_limits/R_min", R_min, 2.0);

        priv_nh.param("deadband_velocity", deadband_velocity, 0.1);

        priv_nh.param("rate_limiter/dec_min_for_lin_vel", dec_min_v, -0.1);
        priv_nh.param("rate_limiter/a_acc_for_lin_vel", a_acc_v, 0.5);
        priv_nh.param("rate_limiter/b_acc_for_lin_vel", b_acc_v, 3.0);
        priv_nh.param("rate_limiter/a_dec_for_lin_vel", a_dec_v, -0.5);
        priv_nh.param("rate_limiter/b_dec_for_lin_vel", b_dec_v, -15.0);
        priv_nh.param("rate_limiter/acc_max_for_lin_vel", acc_max_v, 1.0);
        priv_nh.param("rate_limiter/dec_max_for_lin_vel", dec_max_v, -1.0);
        priv_nh.param("rate_limiter/smoothing_factor_for_lin_vel", smoothing_factor_v, 20.0);

        priv_nh.param("rate_limiter/dec_min_for_ang_vel", dec_min_w, -0.1);
        priv_nh.param("rate_limiter/a_acc_for_ang_vel", a_acc_w, 0.5);
        priv_nh.param("rate_limiter/b_acc_for_ang_vel", b_acc_w, 3.0);
        priv_nh.param("rate_limiter/a_dec_for_ang_vel", a_dec_w, -0.5);
        priv_nh.param("rate_limiter/b_dec_for_ang_vel", b_dec_w, -15.0);
        priv_nh.param("rate_limiter/acc_max_for_ang_vel", acc_max_w, 1.0);
        priv_nh.param("rate_limiter/dec_max_for_ang_vel", dec_max_w, -1.0);
        priv_nh.param("rate_limiter/smoothing_factor_for_ang_vel", smoothing_factor_w, 20.0);

        priv_nh.param("dt", dt_, 0.02);

        priv_nh.param("moving_velocity/v_moving_ss", v_moving_ss, 0.5);
        priv_nh.param("moving_velocity/v_moving", v_moving, 0.4);
        priv_nh.param("moving_velocity/v_stopped", v_stopped, 0.01);

        // Initialize other member variables and set up subscriptions and publishers
        sub_cmd_vel_ = node.subscribe(std::string(topic_ns + "/cmd_vel"), 10, &VelocityController::cmdVelCallback, this);
        sub_odom_ = node.subscribe(std::string(topic_ns + "/odom"), 10, &VelocityController::odomCallback, this);
        sub_moboility_msg_ = node.subscribe(std::string(topic_ns + "/cmd_mobility"), 10, &VelocityController::cmdMobilityCallback, this);
        sub_brakes_ = node.subscribe(std::string(topic_ns + "/brake_command"), 10, &VelocityController::brakeCallback, this);
        pub_cmd_trq_ = node.advertise<deeporange14_msgs::TorqueCmdStamped>(std::string(topic_ns + "/cmd_trq"), 10);
        pub_cmd_vel_reprojected_ = node.advertise<geometry_msgs::Twist>(std::string(topic_ns + "/cmd_vel_reprojected"), 10);
        pub_remap_state_=node.advertise<std_msgs::UInt8>(std::string(topic_ns+"/remapping_state"),10);
      	pub_pid_components_=node.advertise<deeporange14_msgs::PIDComponentsMsg>(std::string(topic_ns+"/pid_components"),10);
        timer_ = node.createTimer(ros::Duration(1.0 / 50.0), &VelocityController::publishTorques, this);
        pub_cmd_vel_cntrl = node.advertise<deeporange14_msgs::CmdVelCntrl>(std::string(topic_ns + "/cmd_vel_cntrl"), 10);

        // Other member variable initializations
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

    }
    VelocityController::~VelocityController(){}
    //defining the odom callback -- to be used by the controller as a feedback of the actual vehicle velocity
    void VelocityController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        vehLinX_=msg->twist.twist.linear.x;
        vehAngZ_=msg->twist.twist.angular.z;
    }
    //defining the brake signal callback -- used to determine when the the controller needs to be used to control the vehicle speeds
    void VelocityController::brakeCallback(const std_msgs::Bool::ConstPtr& msg){
        brake_engage_=msg->data;
    }
    //calback for reading the current state of the autonomy state-machine
    void VelocityController::cmdMobilityCallback(const deeporange14_msgs::MobilityMsg::ConstPtr& msg){
        autonomy_state_=msg->au_state;
    }
    
 void VelocityController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
        cmdLinX_=msg->linear.x;
        cmdAngZ_=msg->angular.z;  // +ve for nlopt -ve for mppi
        errLinX_prev_=errLinX_current_;
        errOmega_prev_=errOmega_current_;
        //integrator reset when we move into 'waiting for execution' state -- after a mission is completed or cancelled, we move back to 'startup' state in the state machine, but the transitions to 'wait execution' should happen
        if (autonomy_state_ == AU_3_ROS_MODE_EN){
            errLinX_integral_=0.0;
            errOmega_integral_=0.0;
            tqL_=0.0;
            tqR_=0.0;
            prev_v_=0.0;
            prev_omega_=0.0;
            remapping_state = VEHICLE_STOPPED;
            ROS_WARN("Left: %f, Right: %f",tqL_,tqR_);
            // ROS_INFO("Velocity error integral: %f, Curvature error integral: %f",errLinX_integral_,errOmega_integral_);
            // prev_time_=(ros::Time::now().toSec()+ros::Time::now().toNSec()*1e-9);
        }
        else if (autonomy_state_ == AU_5_ROS_CONTROLLED || autonomy_state_ == AU_4_DISENGAGING_BRAKES ){
            //letting the controller kick in only when we move in the appropriate autonomy state
            //rate limiting the linear velocity and the curvature
            //velocity reprojection on the commanded velocities
	    ROS_INFO("Angular Velocity from Local Planner to velocity projections is : %f",cmdAngZ_);
            // this->linearVelocityReprojection(cmdLinX_,cmdAngZ_);
            // this->twistReprojection(cmdLinX_,cmdAngZ_);

            double reprojection_x = cmdLinX_;
            double reprojection_w = cmdAngZ_;

            ROS_INFO("Angular Velocity from velocity projections to rate limiter is : %f",cmdAngZ_);
            this->rateLimiter_LinX(prev_v_,cmdLinX_);
            this->rateLimiter_AngZ(prev_omega_,cmdAngZ_);

            double ratelimiter_x = cmdLinX_;
            double ratelimiter_w = cmdAngZ_;

            deeporange14_msgs::CmdVelCntrl Cntrlmsg;
            Cntrlmsg.reprojection_x = reprojection_x;
            Cntrlmsg.reprojection_w = reprojection_w;
            Cntrlmsg.ratelimiter_x = ratelimiter_x;
            Cntrlmsg.ratelimiter_w = ratelimiter_w;

            pub_cmd_vel_cntrl.publish(Cntrlmsg);

            ROS_INFO("Angular Velocity From Rate Limter is : %f",cmdAngZ_);
            cmd_turn_curvature_=(cmdLinX_!=0.0 && cmdAngZ_!=0.0)?(cmdAngZ_/cmdLinX_):0.0;

            //publishing these results on a different topic to compare the changes
            geometry_msgs::Twist cmd_vel_reprojected_;
            cmd_vel_reprojected_.linear.x=cmdLinX_;
            cmd_vel_reprojected_.angular.z=cmdAngZ_;
            pub_cmd_vel_reprojected_.publish(cmd_vel_reprojected_); 
            
            tqDiff_ff_=(x0_*cmd_turn_curvature_)/(1+x1_*std::abs(cmd_turn_curvature_));
            tqCom_ff_=((cmdLinX_>0)-(cmdLinX_<0))*(a_*std::abs(cmdLinX_)+b_);
            //current errors
            errLinX_current_=cmdLinX_-vehLinX_;
            errOmega_current_=cmdAngZ_-vehAngZ_;
            ROS_INFO("Error in Angular velocities is %f",errOmega_current_);
            //current derivatives
            errLinX_derivative_=(errLinX_current_-errLinX_prev_)/dt_;
            errOmega_derivative_=(errOmega_current_-errOmega_prev_)/dt_;
            // discrete controller output
            tqComm_PID_=(kP_linX_*errLinX_current_)+(kI_linX_*errLinX_integral_)+(kD_linX_*errLinX_derivative_);
            tqDiff_PID_=(kP_omega_*errOmega_current_)+(kI_omega_*errOmega_integral_)+(kD_omega_*errOmega_derivative_);
            //publishing individual components of the PID on the PIDComponentsMsg
            deeporange14_msgs::PIDComponentsMsg pid_components_msg_;
            pid_components_msg_.P_Vx=kP_linX_*errLinX_current_;
            pid_components_msg_.I_Vx=kI_linX_*errLinX_integral_;
            pid_components_msg_.D_Vx=kD_linX_*errLinX_derivative_;
            pid_components_msg_.P_Wz=kP_omega_*errOmega_current_;
            
            
            
            pid_components_msg_.I_Wz=kI_omega_*errOmega_integral_;
            pid_components_msg_.D_Wz=kD_omega_*errOmega_derivative_;
            pub_pid_components_.publish(pid_components_msg_);

            //printing the components
            //ROS_INFO("PVx: %f, IVx: %f, DVx: %f",kP_linX_*errLinX_current_,kI_linX_*errLinX_integral_,kD_linX_*errLinX_derivative_);
            //ROS_INFO("PWz: %f, IWz: %f, DWz: %f",kP_omega_*errOmega_current_,kI_omega_*errOmega_integral_,kD_omega_*errOmega_derivative_);

            // feedforward + PID output
            tqDiff_=tqDiff_ff_ + tqDiff_PID_;
            tqComm_=tqCom_ff_ + tqComm_PID_;
            // ROS_WARN("ff: %f, pid: %f",tqCom_ff_,tqComm_PID_);
            //splitting the common and differential torque into left and right torque
            
            tqL_=tqComm_-tqDiff_;
            tqR_=tqComm_+tqDiff_;
            //anti-windup behavior
            if (((tqL_ >= tq_Max_ || tqR_ >= tq_Max_) || ((tqL_ <= tq_Min_) || (tqR_ <= tq_Min_)))){
                //ROS_WARN("Saturated Left: %f, Saturated Right: %f",tqL_,tqR_);
                if (tqComm_PID_*errLinX_current_ > 0){
                    //stop integration for common torque for the next timestep, hence only curvature integral updated
                    errOmega_integral_+=errOmega_current_*dt_;
                }
                else{
                    //resume integration for common torque the next timestep, hence both integrals updated
                    errLinX_integral_+=errLinX_current_*dt_;
                    errOmega_integral_+=errOmega_current_*dt_;
                }
                //publish the torques at saturation limit in either case
                tqL_=std::max((std::min(tqL_,tq_Max_)),tq_Min_);
                tqR_=std::max((std::min(tqR_,tq_Max_)),tq_Min_);
            }
            else{
                ROS_WARN("Unsaturated Left: %f, Unsaturated Right: %f",tqL_,tqR_);
                // only update the error integrals, and NOT limit the torques since we haven't reached the limit
                errLinX_integral_+=errLinX_current_*dt_;
                errOmega_integral_+=errOmega_current_*dt_;
            }
            // ROS_INFO("Curvature error integral: %f",errOmega_integral_);
        }
        else{
            //for every other state we publish zero torques because the brakes are still enabled and we don't want the velocity controller to kick in
            tqL_=0.0;
            tqR_=0.0;
        }
    }
    void VelocityController::linearVelocityReprojection( double& v,  double& w){

        switch(remapping_state){
            case VEHICLE_STOPPED:
            {   
                remapping_state_msg_.data=0;
                pub_remap_state_.publish(remapping_state_msg_);

                ROS_INFO("VEHICLE STOPPED");
                if (std::abs(v) >0 || std::abs(w)>0){
                    //move into the accelerating state
                    v =std::max(v,v_moving_ss); //v_accelerating is the minimum steady state velocity that we want the vehicle to move forward with
                    remapping_state=VEHICLE_ACCELERATING;
                    break;
                }
                else{
                    v =0;
                    //do nothing, remain in this state
                    break;
                }
            }
            case VEHICLE_ACCELERATING:{
                
                remapping_state_msg_.data=1;
                pub_remap_state_.publish(remapping_state_msg_);
                ROS_INFO("VEHICLE ACCELERATING");
                if (std::abs(vehLinX_) >= v_moving){
                    remapping_state=VEHICLE_MOVING;
                    break;
                }
                else{
                    //do nothing, remain in this state
                    v =std::max(v,v_moving_ss);
                    break;
                }
            }
            case VEHICLE_MOVING:{
                remapping_state_msg_.data=2;
                pub_remap_state_.publish(remapping_state_msg_);
                ROS_INFO("VEHICLE MOVING");
                //v stays the same and we do not need to reproject
                if (std::abs(vehLinX_) <= v_stopped){
                    v=0;
                    remapping_state=VEHICLE_STOPPED;
                    break;
                }
                else{
                    //do nothing, remain in this state
                    break;
                }
            }
        }
    }
    
    void VelocityController::twistReprojection(double &v, double &w){
    // Function to reproject commanded stack velocities onto a velocity
    // space that is executable by the Deep Orange 14 vehicle

    // Calculating commanded radius of curvature and lateral acceleration
    double R = (w != 0)? v/w : std::numeric_limits<double>::infinity();
    double lat_acc = v * w;

    // Applying linear velocity limits to bring it into correct zone
    if (v < min_velocity){
      v = min_velocity;  // limited to max forward
    }
    if (v > max_velocity){
      v = max_velocity;  // limited to max forward
    }

    // First Zone: Deadband (If small linear velocities -> go linear only and make angular velocities zero to avoid stall)
    if(fabs(v) <= deadband_velocity){
      w = 0;
    }
    // Second zone: Commanded curvature should not exceed max curvature
    else if(fabs(v) <= v_sz && fabs(R) < R_min) {
      w = v/R_min * (w/fabs(w));
      if(!isfinite(w)){
        ROS_WARN("Deep Orange: w is not finite within reprojection");
      }
    }
    // Third zone: Commanded lateral acceleration should not exceed max
    else if(fabs(v) <= max_velocity && fabs(lat_acc) > lat_acc_max) {
      // Maintaining same curvature but reducing v and w to lie on v*w = lat_acc_max
      v = sqrt(lat_acc_max*fabs(R)) * (v/fabs(v));
      if(!isfinite(v)){
        ROS_WARN("Deep Orange: v is not finite within reprojection");
      }
      
      w = sqrt(lat_acc_max/fabs(R)) * (w/fabs(w));
      if(!isfinite(w)){
        ROS_WARN("Deep Orange: w is not finite within reprojection");
      }
    }
    // TODO: State machine for accelerating, decelerating, stopped
  }

    void VelocityController::rateLimiter_LinX(double &prev_u_, double &u_){
    /*
    % if positive command
dec_min = -0.1;
if sign(u) > 0
    % if positive last
    if sign(ulast) >= 0
        rmax = min([a_acc+b_acc*abs(ulast),acc_max]);
        rmin = max([a_dec+b_dec*abs(ulast),dec_max]);
    % if negative last
    else
        rmax = min([-(a_dec+b_dec*abs(ulast)),-dec_max]);
        rmin = max([-(a_acc+b_acc*abs(ulast)),-acc_max]);
    end
% if negative command
elseif sign(u) < 0
    % if negative last
    if sign(ulast) < 0
        rmax = min([-(a_dec+b_dec*abs(ulast)),-dec_max]);
        rmin = max([-(a_acc+b_acc*abs(ulast)),-acc_max]);
    % if positive last
    else
        rmax = min([a_acc+b_acc*abs(ulast),acc_max]);
        rmin = max([a_dec+b_dec*abs(ulast),dec_max]);
    end
% if zero command
else
    % if positive last
    if sign(ulast) > 0
        rmax = 0;
        rmin =  max(-max([a_dec+b_dec*ulast,sm*ulast])+dec_min,dec_max);        double dec_min;
        double a_acc;
        double b_acc;
        double a_dec;
        double b_dec;
        double acc_max;
        double dec_max;
        double rmin;
        double rmax;
        double smoothing_factor;
    end
end
    */
   //if positive command
   if ( u_ > 0 ){
    if (prev_u_ >= 0){
    rmax_v = std::min(a_acc_v+b_acc_v*std::abs(prev_u_),acc_max_v);
    rmin_v = std::max(a_dec_v+b_dec_v*std::abs(prev_u_),dec_max_v);          
    }
    else{
        rmax_v = std::min(-(a_dec_v+b_dec_v*std::abs(prev_u_)),-dec_max_v);
        rmin_v = std::max(-(a_acc_v+b_acc_v*std::abs(prev_u_)),-acc_max_v);
    }
   }
   else if (u_ < 0){
        if (prev_u_ < 0){
            rmax_v = std::min(-(a_dec_v+b_dec_v*std::abs(prev_u_)),-dec_max_v);
            rmin_v = std::max(-(a_acc_v+b_acc_v*std::abs(prev_u_)),-acc_max_v);
        }

        else{
            rmax_v = std::min(a_acc_v+b_acc_v*std::abs(prev_u_),acc_max_v);
            rmin_v = std::max(a_dec_v+b_dec_v*std::abs(prev_u_),dec_max_v);
        }
   }
   else{
       if (prev_u_ >0){
           rmax_v = 0;
           rmin_v = std::max(-std::max(a_dec_v+b_dec_v*prev_u_,smoothing_factor_v*prev_u_)+dec_min_v,dec_max_v);
        }
        else{
            rmax_v = std::min(std::max(a_dec_v+b_dec_v*prev_u_,-smoothing_factor_v*prev_u_)-dec_min_v,-dec_max_v);
            rmin_v = 0;
        }
    }
        //limits the rate of change of the incoming velocity and curvature commands
    // current_time_=(ros::Time::now().toSec()+ros::Time::now().toNSec()*1e-9);
    // dt_=current_time_-prev_time_;
    // prev_time_=current_time_;
    //linear velocity rate limiter
    double rate_u_ =(u_ - prev_u_)/dt_;
    double allowable_rate_u_=std::max(std::min(rate_u_,rmax_v),rmin_v);
    u_ =prev_u_+allowable_rate_u_*dt_;
    prev_u_= u_;

    }

    // Angular Velocity Rate Limiter


    void VelocityController::rateLimiter_AngZ(double &prev_w_, double &w_){
        //if positive command
        if ( w_ > 0 ){
         if (prev_w_ >= 0){
        rmax_w = std::min(a_acc_w+b_acc_w*std::abs(prev_w_),acc_max_w);
        rmin_w = std::max(a_dec_w+b_dec_w*std::abs(prev_w_),dec_max_w);          
        }
         else{
            rmax_w = std::min(-(a_dec_w+b_dec_w*std::abs(prev_w_)),-dec_max_w);
            rmin_w = std::max(-(a_acc_w+b_acc_w*std::abs(prev_w_)),-acc_max_w);
        }
        }
        else if (w_ < 0){
            if (prev_w_ < 0){
                rmax_w = std::min(-(a_dec_w+b_dec_w*std::abs(prev_w_)),-dec_max_w);
                rmin_w = std::max(-(a_acc_w+b_acc_w*std::abs(prev_w_)),-acc_max_w);
            }
            else{
                rmax_w = std::min(a_acc_w+b_acc_w*std::abs(prev_w_),acc_max_w);
                rmin_w = std::max(a_dec_w+b_dec_w*std::abs(prev_w_),dec_max_w);
            }
        }
        else{
            if (prev_w_ >0){
                rmax_w = 0;
                rmin_w = std::max(-std::max(a_dec_w+b_dec_w*prev_w_,smoothing_factor_w*prev_w_)+dec_min_w,dec_max_w);
            }
            else{
                rmax_w = std::min(std::max(a_dec_w+b_dec_w*prev_w_,-smoothing_factor_w*prev_w_)-dec_min_w,-dec_max_w);
                rmin_w = 0;
            }
        }
             //limits the rate of change of the incoming velocity and curvature commands
         // current_time_=(ros::Time::now().toSec()+ros::Time::now().toNSec()*1e-9);
         // dt_=current_time_-prev_time_;
         // prev_time_=current_time_;
         //linear velocity rate limiter
        double rate_w_ =(w_ - prev_w_)/dt_;
        double allowable_rate_w_=std::max(std::min(rate_w_,rmax_w),rmin_w);
        ROS_INFO("CURRENT RATE: %f, Maximum Rate Limiter ANG: %f,Minimum Rate Limiter ANG: %f",rate_w_,rmax_w,rmin_w);

        w_ =prev_w_+allowable_rate_w_*dt_;
        prev_w_= w_;

    }


    void VelocityController::publishTorques(const ros::TimerEvent& event){
        deeporange14_msgs::TorqueCmdStamped trq_cmd_;
        trq_cmd_.header.stamp = ros::Time::now();
        trq_cmd_.tqL_cmd = tqL_;
        trq_cmd_.tqR_cmd = tqR_;
        pub_cmd_trq_.publish(trq_cmd_);
    }
}
//     int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "deeporange14_velocity_controller");
//   ros::NodeHandle node;
//   ros::NodeHandle priv_nh("~");
//   // create the velocity controller object
//   deeporange14::VelocityController n_velocity_controller(node, priv_nh);
//   // handle callbacks until shut down
//   ros::spin();
//   return 0;
// }

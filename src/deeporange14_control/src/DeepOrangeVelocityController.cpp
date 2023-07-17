#include <deeporange14_control/DeepOrangeVelocityController.h>

namespace deeporange14{
    VelocityController::VelocityController(ros::NodeHandle &node, ros::NodeHandle &priv_nh){
        sub_cmd_vel_=node.subscribe(std::string(topic_ns+"/cmd_vel"),10,&VelocityController::cmdVelCallback,this);
        sub_odom_=node.subscribe(std::string(topic_ns+"/odom"),10,&VelocityController::odomCallback,this);
        sub_moboility_msg_=node.subscribe(std::string(topic_ns+"/cmd_mobility"),10,&VelocityController::cmdMobilityCallback,this);
        sub_brakes_=node.subscribe(std::string(topic_ns+"/brake_command"),10,&VelocityController::brakeCallback,this);
        pub_cmd_trq_=node.advertise<deeporange14_msgs::TorqueCmdStamped>(std::string(topic_ns+"/cmd_trq"),10);
        pub_cmd_vel_reprojected_=node.advertise<geometry_msgs::Twist>(std::string(topic_ns+"/cmd_vel_reprojected"),10);
        timer_ = node.createTimer(ros::Duration(1.0 / 50.0), &VelocityController::publishTorques, this);
        //member variables -- velocities (commanded and platform)
        cmdLinX_=0.0;
        cmdAngZ_=0.0;
        vehLinX_=0.0;
        vehAngZ_=0.0;
        // member variables -- feedforward and PID torques
        tqDiff_ff_=0.0;
        tqCom_ff_=0.0;
        tqDiff_PID_=0.0;
        tqComm_PID_=0.0;
        tqDiff_=0.0;
        tqComm_=0.0;
        tqL_=0.0;
        tqR_=0.0;
        //member variables -- PID controller gains and errors
        errLinX_current_=0.0;
        errLinX_prev_=0.0;
        errLinX_integral_=0.0;
        errLinX_derivative_=0.0;
        errOmega_current_=0.0;
        errOmega_prev_=0.0;
        errOmega_integral_=0.0;
        errOmega_derivative_=0.0;
        kP_linX_=200.0;
        kI_linX_=20.0;
        kD_linX_=0.0;
        kP_omega_=250.0;
        kI_omega_=20.0;
        kD_omega_=0.0;
        //feedforward terms
        x0_=1800.0;
        x1_=7.8;
        a_=20.93;
        cmd_turn_curvature_=0.0;
        // odom_turn_curvature_=0.0;
        //torque limits
        tq_Max_=280.0;
        tq_Min_=-280.0;
        //rate limits
        max_acceleration_limit_=0.5*9.80;   //[m/s]/s
        min_acceleration_limit_=-0.3*9.80;  //[m/s]/s
        max_alpha_limit_=1.0;               //[rad/s]/s
        min_alpha_limit_=-1.0;              //[rad/s]/s
        // curvature_rate_limit_=2.0;       //[1/m]/s
        prev_v_=0.0;
        prev_omega_=0.0;
        dt_=0.0;

        trackwidth=2.60;
        max_velocity=10.0;
        min_velocity=0.5;
        max_omega=2.0;
        min_omega=0.5;
        R_min = trackwidth/2;   // chosen so that tracks do not turn in opposite directions at max curvature
        v_sz = max_omega*R_min; // v_sz = intersection of max curvature line and max lateral acceleration curve
        lat_acc_max = max_omega*v_sz;
        autonomy_state_=AU_1_STARTUP;

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
    
    // defining the cmd_vel callback to update the torque message
    void VelocityController::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg){
        cmdLinX_=msg->linear.x;
        cmdAngZ_=msg->angular.z;
        cmd_turn_curvature_=(cmdLinX_!=0)?(cmdAngZ_/cmdLinX_):std::numeric_limits<double>::infinity();
        errLinX_prev_=errLinX_current_;
        errOmega_prev_=errOmega_current_;
        //integrator reset when we move into 'waiting for execution' state -- after a mission is completed or cancelled, we move back to 'startup' state in the state machine, but the transitions to 'wait execution' should happen
        if (autonomy_state_ == AU_3_WAIT_EXECUTION){
            errLinX_integral_=0.0;
            errOmega_integral_=0.0;
            // ROS_INFO("Velocity error integral: %f, Curvature error integral: %f",errLinX_integral_,errOmega_integral_);
            prev_time_=(ros::Time::now().toSec()+ros::Time::now().toNSec()*1e-9);
        }
        else if (autonomy_state_ == AU_6_COMMAND_TORQUES){
            //letting the controller kick in only when we move in the appropriate autonomy state
            //rate limiting the linear velocity and the curvature
            this->rateLimiter(prev_v_,prev_omega_,cmdLinX_,cmdAngZ_);
            //velocity reprojection on the commanded velocities
            if (cmdLinX_==0.0 && cmdAngZ_==0.0){
                //do nothing -- this is when the local planner has not kicked in and the stack is still sending out zero velocities -- redundant but ok
            }
            else{
                this->velocityReprojection(cmdLinX_,cmdAngZ_);
                //publishing these results on a different topic to compare the changes
                geometry_msgs::Twist cmd_vel_reprojected_;
                cmd_vel_reprojected_.linear.x=cmdLinX_;
                cmd_vel_reprojected_.angular.z=cmdAngZ_;
                pub_cmd_vel_reprojected_.publish(cmd_vel_reprojected_);
            }
            // odom_turn_curvature_=(vehLinX_!=0)?vehAngZ_/vehLinX_:std::numeric_limits<double>::infinity();
            tqDiff_ff_=(x0_*cmd_turn_curvature_)/(1+x1_*std::abs(cmd_turn_curvature_));
            tqCom_ff_=((cmdLinX_>=0)-(cmdLinX_<0))*(a_*std::abs(cmdLinX_)+20.0);
            //current errors
            errLinX_current_=cmdLinX_-vehLinX_;
            errOmega_current_=cmdAngZ_-vehAngZ_;
            //current derivatives
            errLinX_derivative_=(errLinX_current_-errLinX_prev_)/dt_;
            errOmega_derivative_=(errOmega_current_-errOmega_prev_)/dt_;
            // discrete controller output
            tqComm_PID_=(kP_linX_*errLinX_current_)+(kI_linX_*errLinX_integral_)+(kD_linX_*errLinX_derivative_);
            tqDiff_PID_=(kP_omega_*errOmega_current_)+(kI_omega_*errOmega_integral_)+(kD_omega_*errOmega_derivative_);
            // feedforward + PID output
            tqDiff_=tqDiff_ff_ + tqDiff_PID_;
            tqComm_=tqCom_ff_ + tqComm_PID_;
            //splitting the common and differential torque into left and right torque
            tqL_=tqComm_+tqDiff_;
            tqR_=tqComm_-tqDiff_;
            //anti-windup behavior
            if (((tqL_ >= tq_Max_ || tqR_ >= tq_Max_) || ((tqL_ <= tq_Min_) || (tqR_ <= tq_Min_)))){
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
    void VelocityController::velocityReprojection(double &v, double &w){
        double R = (w != 0)? v/w : std::numeric_limits<double>::infinity();
        double lat_acc = v * w;
        if (fabs(v) <= min_velocity){
        w = 0;
        }
        else if (fabs(v) <= v_sz && fabs(R) < R_min) {
        w = v/R_min * (w/fabs(w));
        }
        else if(fabs(v) <= max_velocity && fabs(lat_acc) > lat_acc_max) {
            // Maintaining same curvature but reducing v and w to lie on v*w = lat_acc_max
            v = sqrt(lat_acc_max*fabs(R)) * (v/fabs(v));
            w = sqrt(lat_acc_max/fabs(R)) * (w/fabs(w));
        }
    }
    void VelocityController::rateLimiter(double &prev_v_, double &prev_omega_, double &v, double &omega){
        //limits the rate of change of the incoming velocity and curvature commands
        current_time_=(ros::Time::now().toSec()+ros::Time::now().toNSec()*1e-9);
        dt_=current_time_-prev_time_;
        prev_time_=current_time_;
        //linear velocity rate limiter
        double rate_v_=(v-prev_v_)/dt_;
        double allowable_rate_v_=std::max(std::min(rate_v_,max_acceleration_limit_),min_acceleration_limit_);
        v=prev_v_+allowable_rate_v_*dt_;
        prev_v_=v;
        //curvature rate limiter
        double rate_omega_=(omega-prev_omega_)/dt_;
        double allowable_rate_omega_=std::max(std::min(rate_omega_,max_alpha_limit_),min_alpha_limit_);
        omega=prev_omega_+allowable_rate_omega_*dt_;
        prev_omega_=omega;
    }
    void VelocityController::publishTorques(const ros::TimerEvent& event){
        deeporange14_msgs::TorqueCmdStamped trq_cmd_;
        trq_cmd_.header.stamp = ros::Time::now();
        trq_cmd_.left_torque_cmd = tqL_;
        trq_cmd_.right_torque_cmd = tqR_;
        pub_cmd_trq_.publish(trq_cmd_);
    }
}
    int main(int argc, char **argv)
{
  ros::init(argc, argv, "deeporange14_velocity_controller");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");
  // create the velocity controller object
  deeporange14::VelocityController n_velocity_controller(node, priv_nh);
  // handle callbacks until shut down
  ros::spin();
  return 0;
}
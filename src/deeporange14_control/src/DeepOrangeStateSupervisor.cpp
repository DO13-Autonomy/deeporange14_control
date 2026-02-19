/*
Implement a state machine based on information from both the Raptor and Phoenix autonomy stack
*/

#include <string>
#include <deeporange14_control/DeepOrangeStateSupervisor.h>

namespace deeporange14 {
DeepOrangeStateSupervisor::DeepOrangeStateSupervisor(ros::NodeHandle &node, ros::NodeHandle &priv_nh) {
  /* subscribers and publishers */

  // from Phoenix (ROS)
  sub_mission_status_ = node.subscribe(std::string(topic_ns_ + "/mission/status"), 10,
                            &DeepOrangeStateSupervisor::getMissionStatus, this, ros::TransportHints().tcpNoDelay(true));

  sub_stop_ros_ = node.subscribe(std::string(topic_ns_ + "/stop_ros"), 10, &DeepOrangeStateSupervisor::getStopRos, this,
                             ros::TransportHints().tcpNoDelay(true));

  sub_cmd_vel_ = node.subscribe(std::string(topic_ns_ + "/cmd_vel"), 10, &DeepOrangeStateSupervisor::getCmdVel, this,
                            ros::TransportHints().tcpNoDelay(true)); 

  // from Raptor (CAN-ROS interface)
  sub_au_meas_ = node.subscribe(std::string(topic_ns_ + "/au_meas"), 10,
                              &DeepOrangeStateSupervisor::getMeasurements, this, ros::TransportHints().tcpNoDelay(true));

  // to Phoenix (ROS) -- Phoenix doesn't use these topics, but make them available for monitoring purposes
  pub_vx_meas_ = node.advertise<std_msgs::Float32>(std::string(topic_ns_ + "/meas_vx"), 10, this);
  pub_curv_meas_ = node.advertise<std_msgs::Float32>(std::string(topic_ns_ + "/meas_curv"), 10, this);
  pub_wx_meas_ = node.advertise<std_msgs::Float32>(std::string(topic_ns_ + "/meas_wx"), 10, this);

  // to Raptor (CAN-ROS interface)
  pub_au_cmd_ = node.advertise<deeporange14_msgs::AutonomyCommandMsg>(std::string(topic_ns_ + "/au_cmd"), 10, this);

  /* Initiate ROS State in the Default state and false booleans to ensure transition only when it actually receives a 
     True. This state will be published by the timer object at 50Hz update_freq. */
  dbw_ros_mode = false;

  vx_meas_ = 0.0;
  vx_cmd_ = 0.0;

  curv_meas_ = 0.0;
  curv_cmd_ = 0.0;
  
  wx_meas_calc_ = 0.0;
  
  dbw_state_ = DBW_0_AUTO_OFF;
  au_state_ = AU_0_NO_HEARTBEAT;
  prev_au_state_ = AU_0_NO_HEARTBEAT;

  priv_nh.getParam("cmd_recv_timeout", cmd_recv_timeout_s_);
  priv_nh.getParam("raptor_timeout", raptor_timeout_s_);
  priv_nh.getParam("update_freq", update_freq_hz_);

  fault_delay_s_ = 20;  // after fault, wait 20 sec before state transition
  fault_delay_ct_ = fault_delay_s_ * update_freq_hz_;  // threshold for counter
  fault_delay_ = 0;  // delay counter

  last_cmd_recv_time_ = 0.0;
  last_raptor_hb_time_ = 0.0;
  ros_stop_time_ = 0.0;

  raptor_fault_ = true;
  stack_fault_ = true;
  stop_ros_ = false;

  mppi_mission_status_ = 0;

  // set up timer to publish autonomy commands
  timer_ = node.createTimer(ros::Duration(1.0 / update_freq_hz_), &DeepOrangeStateSupervisor::supervisorControlUpdate, this);
  meas_timer_ = node.createTimer(ros::Duration(1.0 / update_freq_hz_), &DeepOrangeStateSupervisor::pubMeasurements, this);
}
DeepOrangeStateSupervisor::~DeepOrangeStateSupervisor() {}

// publish the measurements to ROS topics
// although these aren't used by Phoenix, it may be helpful to have them available for monitoring purposes
void DeepOrangeStateSupervisor::pubMeasurements(const ros::TimerEvent& event){
  std_msgs::Float32 meas_msg;

  meas_msg.data = vx_meas_;
  pub_vx_meas_.publish(meas_msg);
  
  meas_msg.data = curv_meas_;
  pub_curv_meas_.publish(meas_msg);

  meas_msg.data = wx_meas_calc_;
  pub_wx_meas_.publish(meas_msg);
}

void DeepOrangeStateSupervisor::getMeasurements(const deeporange14_msgs::AutonomyMeasurementMsg::ConstPtr& msg) {
  // unpack the AutonomyMeasurement message to get information from the Raptor
  float vx_tmp = msg->vx_meas;
  float curv_tmp = msg->curv_meas;

  // speed and curvature must be (1) multiplied by the factor and (2) clamped by the limits in the message
  vx_meas_ = std::min(std::max(vx_tmp * msg->VX_FACTOR, msg->VX_MIN), msg->VX_MAX);
  curv_meas_ = std::min(std::max(curv_tmp * msg->CURV_FACTOR, msg->CURV_MIN), msg->CURV_MAX);
  wx_meas_calc_ = curv_meas_ * vx_meas_;  // TODO - curvature * vx, but need to consider signs (TODO)

  dbw_state_ = msg->dbw_state;  // state can be directly translated from the measurement message
  
  updateStateMachine();  // make sure the state machine is updated each time a new measurement is received
}

// get the commanded velocity and calculate commanded curvature
void DeepOrangeStateSupervisor::getCmdVel(const geometry_msgs::Twist::ConstPtr &msg) {
  last_cmd_recv_time_ = ros::Time::now().toSec();  // use for checking if there is a timeout

  float wx_cmd = msg->angular.z;

  // only command non-zero speed and curvature while the mission is operating
  // TODO - make sure this is correct
  if (au_state_ == AU_4_MISSION_IN_PROGRESS) {
    vx_cmd_ = msg->linear.x;
    curv_cmd_ = wx_cmd / vx_cmd_;  // TODO - get the correct sign
  }
  else {
    vx_cmd_ = 0.0;
    curv_cmd_ = 0.0;
  }
}

// get the timestamp when Phoenix sends a signal to stop ROS
void DeepOrangeStateSupervisor::getStopRos(const std_msgs::Bool::ConstPtr &msg) {
  ros_stop_time_ = ros::Time::now().toSec();
}

// get the MPPI mission status from Phoenix
void DeepOrangeStateSupervisor::getMissionStatus(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
  if (!msg->status_list.empty())
  {
    mppi_mission_status_ = msg->status_list[0].status;
    ROS_DEBUG("MPPI Status: %d", mppi_mission_status_);
  }
}

// timer callback to publish commands regularly
// this runs independent of receiving measurements, as the stack does not use those measurements
// in calculating the commands
void DeepOrangeStateSupervisor::supervisorControlUpdate(const ros::TimerEvent &event) {
  // check for faults in the system:
  //  - stack_fault_ occurs if the Phoenix stop stops sending commands for specified timoeout
  //  - raptor_fault_ occurs if the heartbeat from the Raptor is lost for the timeout period
  stack_fault_ = ros::Time::now().toSec() - last_cmd_recv_time_ > cmd_recv_timeout_s_;
  raptor_fault_ = ros::Time::now().toSec() - last_raptor_hb_time_ > raptor_timeout_s_;

  // check if the stop_ros message has been received "recently" (within the last 5 seconds) from Phoenix
  // reset the flag to 0 if not
  stop_ros_ = (ros::Time::now().toSec() - ros_stop_time_) < 5 ? 1 : 0;

  updateStateMachine();

  // build the AutonomyCommand message, then publish it
  au_cmd_msg_.header.stamp = ros::Time::now();

  // speed and curvature need to be clamped and scaled
  au_cmd_msg_.vx_cmd = std::min(std::max(vx_cmd_, au_cmd_msg_.VX_MIN), au_cmd_msg_.VX_MAX) / au_cmd_msg_.VX_FACTOR;
  au_cmd_msg_.curv_cmd = std::min(std::max(curv_cmd_, au_cmd_msg_.CURV_MIN), au_cmd_msg_.CURV_MAX) / au_cmd_msg_.CURV_FACTOR;

  au_cmd_msg_.au_state = au_state_;

  pub_au_cmd_.publish(au_cmd_msg_);
}

void DeepOrangeStateSupervisor::updateStateMachine() {
  switch (au_state_) {
    case AU_0_NO_HEARTBEAT: {
      ROS_DEBUG("In default state, AU_0_NO_HEARTBEAT");

      prev_au_state_ = AU_0_NO_HEARTBEAT;
      au_state_ = AU_1_WAITING_HEARTBEAT;
      break;
    }
    case AU_1_WAITING_HEARTBEAT: {
      ROS_DEBUG("In startup state, AU_1_WAITING_HEARTBEAT");

      if (!raptor_fault_) {
        ROS_INFO("[AU_1_WAITING_HEARTBEAT]: Raptor handshake is established, transitioning to AU_2_WAITING_HANDOFF");
        prev_au_state_ = AU_1_WAITING_HEARTBEAT;
        au_state_ = AU_2_WAITING_HANDOFF;
        break;
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_1_WAITING_HEARTBEAT]: Raptor handshake not established yet");
        break;
      }
    }
    case AU_2_WAITING_HANDOFF: {
      ROS_DEBUG("In idle state, AU_2_WAITING_HANDOFF");

      if (raptor_fault_) {
        au_state_ = AU_1_WAITING_HEARTBEAT;
        ROS_WARN("[AU_2_WAITING_HANDOFF]: Raptor handshake failed");
        break;
      }
      else if (prev_au_state_ > AU_1_WAITING_HEARTBEAT) {
        // it has returned from fault of below states, add delay
        // TODO - resetting this counter (if it is kept)
        if (fault_delay_ < fault_delay_ct_) {
          fault_delay_++;
          break;
        }
        else {
          prev_au_state_ = AU_1_WAITING_HEARTBEAT;
          break;
        }
      }
      else if (dbw_ros_mode) {
        ROS_INFO("[AU_2_WAITING_HANDOFF]: Transitioning to AU_3_ROS_EN");
        prev_au_state_ = AU_2_WAITING_HANDOFF;
        au_state_ = AU_3_READY_FOR_MISSION;
        break;
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_2_WAITING_HANDOFF]: Ready to enter DBW-ROS mode");
        break;
      }
    }
    case AU_3_READY_FOR_MISSION: {
      ROS_DEBUG("In DBW-ROS state, AU_3_READY_FOR_MISSION");

      if (raptor_fault_) {
        au_state_ = AU_1_WAITING_HEARTBEAT;
        ROS_WARN("[AU_3_READY_FOR_MISSION]: Raptor handshake failed");
        break;
      }
      else if (!dbw_ros_mode) {
        au_state_ = AU_2_WAITING_HANDOFF;
        ROS_WARN("[AU_3_READY_FOR_MISSION]: Exiting DBW-ROS mode");
        break;
      }
      else if (stack_fault_) {
        au_state_ = AU_2_WAITING_HANDOFF;
        ROS_ERROR("[AU_3_READY_FOR_MISSION]: Stack crashed or failed");
        break;
      }
      else if (stop_ros_) {
        au_state_ = AU_2_WAITING_HANDOFF;
        ROS_WARN("[AU_3_READY_FOR_MISSION]: Stop button is pressed");
        break;
      }
      else if (mppi_mission_status_ == 1 || mppi_mission_status_ == 4) {
        prev_au_state_ = AU_3_READY_FOR_MISSION;
        au_state_ = AU_4_MISSION_IN_PROGRESS;
        ROS_INFO("[AU_3_READY_FOR_MISSION]: Local plan ready, disengaging brakes");
        break;
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_3_READY_FOR_MISSION]: Waiting for local plan ready signal");
        break;
      }
    }
    case AU_4_MISSION_IN_PROGRESS: {
      ROS_DEBUG("In disengage brake state, AU_4_MISSION_IN_PROGRESS");

      // Also check from stack if brake_enable command is false from stack,
      // because plan is ready and brakes should be disengaged before moving
      if (raptor_fault_) {
        au_state_ = AU_1_WAITING_HEARTBEAT;
        ROS_WARN("[AU_4_MISSION_IN_PROGRESS]: Raptor handshake failed");
        break;
      }
      else if (!dbw_ros_mode) {
        au_state_ = AU_2_WAITING_HANDOFF;
        ROS_WARN("[AU_4_MISSION_IN_PROGRESS]: Exiting DBW-ROS mode");
        break;
      }
      else if (stack_fault_) {
        au_state_ = AU_2_WAITING_HANDOFF;
        ROS_ERROR("[AU_4_MISSION_IN_PROGRESS]: Stack crashed or failed");
        break;
      }
      else if (mppi_mission_status_ == 3) {
        au_state_ = AU_3_READY_FOR_MISSION;
        ROS_DEBUG("[AU_4_MISSION_IN_PROGRESS]: Phoenix stack mission status changed to %d", mppi_mission_status_);
        break;
      }
      else if (stop_ros_) {
        //  go back to idle
        au_state_ = AU_2_WAITING_HANDOFF;
        ROS_WARN("[AU_4_MISSION_IN_PROGRESS]: Stop button is pressed");
        break;
      }
      else if (speed_state == SPEED_STATE_Ready2Move) {
        prev_au_state_ = AU_4_MISSION_IN_PROGRESS;
        //state = AU_5_ROS_CONTROLLED;
        ROS_INFO("[AU_4_MISSION_IN_PROGRESS]: Brakes disengaged, transitioning to ROS-controlled mode, ready to move");
        break;
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_4_MISSION_IN_PROGRESS]: Waiting for ROS-controlled mode");
        break;
      }
    }
    /*case AU_5_ROS_CONTROLLED: {
      ROS_DEBUG("In ROS-controlled state, AU_5_ROS_CONTROLLED");

      mobilityMsg.tqL_cmd = tqL_cmd_controller;
      mobilityMsg.tqR_cmd = tqR_cmd_controller;
      mobilityMsg.brkL_cmd = 0.0;
      mobilityMsg.brkR_cmd = 0.0;

      if (!raptor_hb_detected) {
        state = AU_1_WAITING_HEARTBEAT;
        ROS_WARN("[AU_5_ROS_CONTROLLED]: Raptor handshake failed");
        break;
      }
      else if (!dbw_ros_mode) {
        state = AU_2_WAITING_HANDOFF;
        ROS_WARN("[AU_5_ROS_CONTROLLED]: Exiting DBW-ROS mode");
        break;
      }
      else if (stack_fault) {
        state = AU_2_WAITING_HANDOFF;
        ROS_ERROR("[AU_5_ROS_CONTROLLED]: Stack crashed or failed ");
        break;
      }
      else if (mppi_status == 3) {
        state = AU_3_READY_FOR_MISSION;
        ROS_DEBUG("WARN: [AU_5_ROS_CONTROLLED]: Phoenix Stack mission status changed to %d", mppi_status);
        break;
      }
      else if (stop_ros) {
        //  go back to idle
        state = AU_2_WAITING_HANDOFF;
        ROS_WARN("[AU_5_ROS_CONTROLLED]: Stop button is pressed");
        break;
      }
      else if (mission_status == "MissionCompleted" || mission_status == "MissionCancelled") {
        prev_au_state_ = AU_5_ROS_CONTROLLED;
        state = AU_3_READY_FOR_MISSION;
        ROS_INFO("[AU_5_ROS_CONTROLLED]: Mission completed or cancelled; returning to AU_3_READY_FOR_MISSION");
        break;
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_5_ROS_CONTROLLED]: Mission Executing");
        break;
      }
    }*/
    default: {
      prev_au_state_ = AU_0_NO_HEARTBEAT;
      ROS_ERROR("Unknown State, shut down");
      break;
    }
  }
}
}  // namespace deeporange14

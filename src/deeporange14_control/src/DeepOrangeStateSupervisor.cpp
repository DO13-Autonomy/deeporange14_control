/*
Implement a state machine based on information from both the Raptor and Phoenix autonomy stack
*/

#include <string>
#include <deeporange14_control/DeepOrangeStateSupervisor.h>

namespace deeporange14 {
DeepOrangeStateSupervisor::DeepOrangeStateSupervisor(ros::NodeHandle &node, ros::NodeHandle &priv_nh) {
  /* subscribers and publishers */

  // ---------- from Phoenix (ROS) ---------- //
  sub_mission_status_ = node.subscribe(std::string(topic_ns_ + "/mission/status"), 10,
                            &DeepOrangeStateSupervisor::getMissionStatus, this, ros::TransportHints().tcpNoDelay(true));

  sub_stop_ros_ = node.subscribe(std::string(topic_ns_ + "/stop_ros"), 10, &DeepOrangeStateSupervisor::getStopRos, this,
                             ros::TransportHints().tcpNoDelay(true));

  sub_cmd_vel_ = node.subscribe(std::string(topic_ns_ + "/cmd_vel"), 10, &DeepOrangeStateSupervisor::getCmdVel, this,
                            ros::TransportHints().tcpNoDelay(true)); 

  // --- from Raptor (CAN-ROS interface) ---- //
  // from Raptor (CAN-ROS interface)
  sub_au_meas_ = node.subscribe(std::string(topic_ns_ + "/au_meas"), 10,
                              &DeepOrangeStateSupervisor::getMeasurements, this, ros::TransportHints().tcpNoDelay(true));

  // ----------- to Phoenix (ROS) ----------- //
  // Phoenix doesn't use these topics, but make them available for monitoring purposes
  pub_vx_meas_ = node.advertise<std_msgs::Float32>(std::string(topic_ns_ + "/meas_vx"), 10, this);
  pub_curv_meas_ = node.advertise<std_msgs::Float32>(std::string(topic_ns_ + "/meas_curv"), 10, this);
  pub_wx_meas_ = node.advertise<std_msgs::Float32>(std::string(topic_ns_ + "/meas_wx"), 10, this);

  // ---- to Raptor (CAN-ROS interface) ----- //
  pub_au_cmd_ = node.advertise<deeporange14_msgs::AutonomyCommandMsg>(std::string(topic_ns_ + "/au_cmd"), 10, this);

  // get parameters
  priv_nh.getParam("cmd_recv_timeout", cmd_recv_timeout_s_);
  priv_nh.getParam("update_freq", update_freq_hz_);

  // set up timer to publish autonomy commands
  timer_ = node.createTimer(ros::Duration(1.0 / update_freq_hz_), &DeepOrangeStateSupervisor::updateControlCommands, this);
  meas_timer_ = node.createTimer(ros::Duration(1.0 / update_freq_hz_), &DeepOrangeStateSupervisor::pubMeasurements, this);

  // initialize private variables
  vx_meas_ = 0.0;
  vx_cmd_ = 0.0;

  curv_meas_ = 0.0;
  curv_cmd_ = 0.0;
  
  wx_meas_calc_ = 0.0;
  
  dbw_state_ = DBW_0_AUTO_OFF;
  au_state_ = AU_0_NO_HEARTBEAT;
  prev_au_state_ = AU_0_NO_HEARTBEAT;

  last_cmd_recv_time_ = 0.0;
  ros_stop_time_ = 0.0;

  stack_fault_ = true;
  stop_ros_ = false;

  mission_status_ = goal_status_dummy_.PENDING;
  mission_running_ = false;
  mission_completed_ = false;
  mission_aborted_ = false;
}

DeepOrangeStateSupervisor::~DeepOrangeStateSupervisor() {}

// publish the measurements to ROS topics
// although these aren't used by Phoenix, it may be helpful to have them available for monitoring purposes
void DeepOrangeStateSupervisor::pubMeasurements(const ros::TimerEvent& event) {
  std_msgs::Float32 meas_msg;

  meas_msg.data = vx_meas_;
  pub_vx_meas_.publish(meas_msg);
  
  meas_msg.data = curv_meas_;
  pub_curv_meas_.publish(meas_msg);

  meas_msg.data = wx_meas_calc_;
  pub_wx_meas_.publish(meas_msg);
}

void DeepOrangeStateSupervisor::pubCommands() {
  // build the AutonomyCommand message, then publish it
  au_cmd_msg_.header.stamp = ros::Time::now();

  // speed and curvature need to be clamped and scaled
  au_cmd_msg_.vx_cmd = std::min(std::max(vx_cmd_, au_cmd_msg_.VX_MIN), au_cmd_msg_.VX_MAX) / au_cmd_msg_.VX_FACTOR;
  au_cmd_msg_.curv_cmd = std::min(std::max(curv_cmd_, au_cmd_msg_.CURV_MIN), au_cmd_msg_.CURV_MAX) / au_cmd_msg_.CURV_FACTOR;

  au_cmd_msg_.au_state = au_state_;

  pub_au_cmd_.publish(au_cmd_msg_);
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

// get the mission status from Phoenix
void DeepOrangeStateSupervisor::getMissionStatus(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
  if (!msg->status_list.empty())
  {
    mission_status_ = msg->status_list[0].status;
    ROS_DEBUG("Mission status: %d", mission_status_);

    updateMissionStatusBools();
  }
}

// update the mission status booleans (for completed, aborted, and running statuses)
// Phoenix only uses 6 of the 10 available statuses, given in the GoalStatus message
void DeepOrangeStateSupervisor::updateMissionStatusBools() {
  switch (mission_status_) {
    case goal_status_dummy_.SUCCEEDED: {
      mission_running_ = false;
      mission_completed_ = true;
      mission_aborted_ = false;
      break;
    }
    case goal_status_dummy_.ACTIVE: {
      mission_running_ = true;
      mission_completed_ = false;
      mission_aborted_ = false;
      break;
    }
    case goal_status_dummy_.ABORTED: {
      mission_running_ = false;
      mission_completed_ = false;
      mission_aborted_ = true;
    }
    case goal_status_dummy_.PENDING:
    case goal_status_dummy_.PREEMPTED:
    case goal_status_dummy_.REJECTED: {
      mission_running_ = false;
      mission_completed_ = false;
      mission_aborted_ = false;
      break;
    }
    default: {
      ROS_WARN("[updateMissionStatusBool]: Unhandled mission status %d, resetting booleans to false", mission_status_);

      mission_running_ = false;
      mission_completed_ = false;
      mission_aborted_ = false;
      break;
    } 
  }
}

// timer callback to publish commands regularly
// this runs independent of receiving measurements, as the stack does not use those measurements
// in calculating the commands
void DeepOrangeStateSupervisor::updateControlCommands(const ros::TimerEvent &event) {
  // check if Phoenix stop stops sending commands for specified timoeout
  stack_fault_ = ros::Time::now().toSec() - last_cmd_recv_time_ > cmd_recv_timeout_s_;

  // check if the stop_ros message has been received "recently" (within the last 5 seconds) from Phoenix
  // reset the flag to 0 if not
  // TODO - do we still need this (communicate to raptor to exit ros mode) after state overhaul?
  stop_ros_ = (ros::Time::now().toSec() - ros_stop_time_) < 5 ? 1 : 0;

  updateStateMachine();

  pubCommands();
}

void DeepOrangeStateSupervisor::updateStateMachine() {
  switch (au_state_) {
    case AU_0_NO_HEARTBEAT: {
      ROS_DEBUG("In state,AU_0_NO_HEARTBEAT");

      prev_au_state_ = AU_0_NO_HEARTBEAT;
      au_state_ = AU_1_WAITING_HEARTBEAT;  // immediatly move to state 1 after node has initialized
      break;
    }
    case AU_1_WAITING_HEARTBEAT: {
      ROS_DEBUG("In state AU_1_WAITING_HEARTBEAT");

      if (dbw_state_ == DBW_1_WAITING_HEARTBEAT) {
        au_state_ = AU_2_WAITING_HANDOFF;  // once the Raptor heartbeat is detected, move to state 2 and wait for ack
        ROS_INFO("[AU_1_WAITING_HEARTBEAT]: Raptor handshake is established, transitioning to AU_2_WAITING_HANDOFF");
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_1_WAITING_HEARTBEAT]: Raptor handshake not established yet");
      }

      prev_au_state_ = AU_1_WAITING_HEARTBEAT;
      break;
    }
    case AU_2_WAITING_HANDOFF: {
      ROS_DEBUG("In state AU_2_WAITING_HANDOFF");

      if (dbw_state_ == DBW_0_AUTO_OFF) {
        au_state_ = AU_1_WAITING_HEARTBEAT;  // go back to state 1 if the Raptor heartbeat is lost
        ROS_WARN("[AU_2_WAITING_HANDOFF]: Raptor handshake failed");
      }
      else if (dbw_state_ == DBW_3_READY_TO_DRIVE) {
        au_state_ = AU_3_READY_FOR_MISSION;  // go to state 3 onces the Raptor is ready to receive mission commands
        ROS_INFO("[AU_2_WAITING_HANDOFF]: Transitioning to AU_3_READY_FOR_MISSION");
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_2_WAITING_HANDOFF]: Ready to enter DBW-ROS mode");
      }

      prev_au_state_ = AU_2_WAITING_HANDOFF;
      break;
    }
    case AU_3_READY_FOR_MISSION: {
      ROS_DEBUG("In state AU_3_READY_FOR_MISSION");

      if (dbw_state_ == DBW_0_AUTO_OFF) {
        au_state_ = AU_1_WAITING_HEARTBEAT;  // go back to state 1 if the Raptor heartbeat is lost
        ROS_WARN("[AU_3_READY_FOR_MISSION]: Raptor handshake failed");
      }
      else if (mission_running_) {
        au_state_ = AU_4_MISSION_IN_PROGRESS;  // go to state 4 when the mission is started
        ROS_INFO("[AU_3_READY_FOR_MISSION]: Transitioning to AU_4_MISSION_IN_PROGRESS");
      }
      else if (mission_completed_ || mission_aborted_ ) {
        au_state_ = AU_2_WAITING_HANDOFF;  // go to state 4 when the mission has ended (or been stopped)
        ROS_INFO("[AU_3_READY_FOR_MISSION]: Mission complete or aborted, transitioning to AU_2_WAITING_HANDOFF");
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_3_READY_FOR_MISSION]: Ready to enter DBW-ROS mode");
      }

      prev_au_state_ = AU_3_READY_FOR_MISSION;
      break;
    }
    case AU_4_MISSION_IN_PROGRESS: {
      ROS_DEBUG("In state AU_4_MISSION_IN_PROGRESS");

      if (dbw_state_ == DBW_0_AUTO_OFF) {
        au_state_ = AU_1_WAITING_HEARTBEAT;  // go back to state 1 if the Raptor heartbeat is lost
        ROS_WARN("[AU_4_MISSION_IN_PROGRESS]: Raptor handshake failed");
      }
      else if (mission_completed_ || mission_aborted_ || stack_fault_) {  // TODO - is it still valid to check for stack_fault_?
        au_state_ = AU_2_WAITING_HANDOFF;  // go to state 4 when the mission has ended (or been stopped)

        if (stack_fault_) {
          ROS_WARN("[AU_4_MISSION_IN_PROGRESS]: Lost communication with Phoenix");
        }
        else {
          ROS_INFO("[AU_4_MISSION_IN_PROGRESS]: Mission complete or aborted, transitioning to AU_2_WAITING_HANDOFF");
        }
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_4_MISSION_IN_PROGRESS]: Ready to enter DBW-ROS mode");
      }

      prev_au_state_ = AU_4_MISSION_IN_PROGRESS;
      break;
    }
    default: {
      au_state_ = AU_0_NO_HEARTBEAT;
      ROS_ERROR("Unknown stated. state machine resetting");
      break;
    }
  }
}
}  // namespace deeporange14

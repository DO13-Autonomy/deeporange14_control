/*
Implement a state machine based on information from both the Raptor and Phoenix autonomy stack
*/

#include <limits>
#include <string>

#include <deeporange14_control/DeepOrangeStateSupervisor.h>

namespace deeporange14 {
DeepOrangeStateSupervisor::DeepOrangeStateSupervisor(ros::NodeHandle &node, ros::NodeHandle &priv_nh) {
  /* get parameters from launch file */
  priv_nh.getParam("vehicle_ns", topic_ns_);
  priv_nh.getParam("au_cmd_topic", topic_au_cmd_);
  priv_nh.getParam("au_meas_topic", topic_au_meas_);
  priv_nh.getParam("cmd_recv_timeout", cmd_recv_timeout_s_);
  priv_nh.getParam("stop_ros_timeout", stop_ros_timeout_s_);
  priv_nh.getParam("update_freq", update_freq_hz_);

  /* subscribers and publishers */

  // ---------- from Phoenix (ROS) ---------- //
  sub_mission_status_ = node.subscribe(std::string(topic_ns_ + "/mission/status"),
                                                   10,
                                                   &DeepOrangeStateSupervisor::getMissionStatus,
                                                   this,
                                                   ros::TransportHints().tcpNoDelay(true));

  sub_stop_ros_ = node.subscribe(std::string(topic_ns_ + "/stop_ros"),
                                             10,
                                             &DeepOrangeStateSupervisor::getStopRos,
                                             this,
                                             ros::TransportHints().tcpNoDelay(true));

  sub_cmd_vel_ = node.subscribe(std::string(topic_ns_ + "/cmd_vel"),
                                10,
                                &DeepOrangeStateSupervisor::getCmdVel,
                                this,
                                ros::TransportHints().tcpNoDelay(true));

  // --- from Raptor (CAN-ROS interface) ---- //
  // from Raptor (CAN-ROS interface)
  sub_au_meas_ = node.subscribe(std::string(topic_ns_ + "/" + topic_au_meas_),
                                10,
                                &DeepOrangeStateSupervisor::getMeasurements,
                                this,
                                ros::TransportHints().tcpNoDelay(true));

  // ----------- to Phoenix (ROS) ----------- //
  // Phoenix doesn't use these topics, but make them available for monitoring purposes
  pub_vx_meas_ = node.advertise<std_msgs::Float32>(std::string(topic_ns_ + "/meas_vx"), 10, this);
  pub_curv_meas_ = node.advertise<std_msgs::Float32>(std::string(topic_ns_ + "/meas_curv"), 10, this);
  pub_wx_meas_ = node.advertise<std_msgs::Float32>(std::string(topic_ns_ + "/meas_wx"), 10, this);
  pub_debug_msg_ = node.advertise<deeporange14_msgs::AutonomyStateSupervisorDebugMsg>(
    std::string(topic_ns_ + "/au_debug"), 10, this);

  // ---- to Raptor (CAN-ROS interface) ----- //
  pub_au_cmd_ = node.advertise<deeporange14_msgs::AutonomyCommandMsg>(
    std::string(topic_ns_ + "/" + topic_au_cmd_), 10, this);

  // set up timer to publish autonomy commands
  cmd_timer_ = node.createTimer(ros::Duration(1.0 / update_freq_hz_),
                            &DeepOrangeStateSupervisor::updateControlCommands,
                            this);

  meas_timer_ = node.createTimer(ros::Duration(1.0 / update_freq_hz_),
                                 &DeepOrangeStateSupervisor::pubMeasurements,
                                 this);

  // set up a timer for the stack_fault_ flag
  // this is a one-shot timer that is not started autonomatically, and asserts
  // the flag when it times out
  stack_fault_timer_ = node.createTimer(ros::Duration(cmd_recv_timeout_s_),
                                        &DeepOrangeStateSupervisor::assertStackFault,
                                        this,
                                        true,
                                        false);

  // set up a timer for the stop_ros_ flag
  // this is a one-shot timer that is not started automatically, and resets
  // the flag when it times out
  stop_ros_timer_ = node.createTimer(ros::Duration(stop_ros_timeout_s_),
                                     &DeepOrangeStateSupervisor::resetStopRos,
                                     this,
                                     true,
                                     false);

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

  if (au_state_ == AU_4_MISSION_IN_PROGRESS) {
    au_cmd_msg_.vx_cmd = vx_cmd_;
    au_cmd_msg_.curv_cmd = curv_cmd_;
  }
  else {
    // command no movement when no mission is in progress
    // this is a bit redundant to getCmdVel, but takes care in case the state has
    // changed since the last time a command was received
    au_cmd_msg_.vx_cmd = 0.0;
    au_cmd_msg_.curv_cmd = 0.0;
  }

  au_cmd_msg_.au_state = au_state_;

  pub_au_cmd_.publish(au_cmd_msg_);
}

void DeepOrangeStateSupervisor::pubDebugMsg() {
  // build the AutonomyStateSupervisorDebug message, then publish it
  au_debug_msg_.header.stamp = ros::Time::now();

  au_debug_msg_.au_state = au_state_;
  au_debug_msg_.dbw_state = dbw_state_;

  au_debug_msg_.vx_cmd = vx_cmd_;
  au_debug_msg_.vx_meas = vx_meas_;

  au_debug_msg_.curv_cmd = curv_cmd_;
  au_debug_msg_.curv_meas = curv_meas_;

  au_debug_msg_.wx_cmd = wx_cmd_;
  au_debug_msg_.wx_meas = wx_meas_calc_;

  au_debug_msg_.stack_fault = stack_fault_;
  au_debug_msg_.stop_ros = stop_ros_;
  au_debug_msg_.mission_running = mission_running_;
  au_debug_msg_.mission_complete = mission_completed_;
  au_debug_msg_.mission_aborted = mission_aborted_;

  pub_debug_msg_.publish(au_debug_msg_);
}

void DeepOrangeStateSupervisor::getMeasurements(const deeporange14_msgs::AutonomyMeasurementMsg::ConstPtr& msg) {
  // unpack the AutonomyMeasurement message to get information from the Raptor
  vx_meas_ = msg->vx_meas;
  curv_meas_ = msg->curv_meas;
  wx_meas_calc_ = curv_meas_ * vx_meas_;

  dbw_state_ = msg->dbw_state;  // state can be directly translated from the measurement message

  updateStateMachine();  // make sure the state machine is updated each time a new measurement is received
}

// get the commanded velocity and calculate commanded curvature
void DeepOrangeStateSupervisor::getCmdVel(const geometry_msgs::Twist::ConstPtr &msg) {
  // stop the timer if it is running, then restart it
  ROS_DEBUG("cmd_vel message received, starting timer");
  stack_fault_timer_.stop();
  stack_fault_timer_.start();

  stack_fault_ = false;

  wx_cmd_ = msg->angular.z;

  // only command non-zero speed and curvature while the mission is operating
  if (au_state_ == AU_4_MISSION_IN_PROGRESS) {
    vx_cmd_ = msg->linear.x;

    // take care not to divide by 0, using epsilon (O(1e-16))
    // TODO - better to use min (O(1e-308))?
    if (std::abs(vx_cmd_) > std::numeric_limits<double>::epsilon()) {
      curv_cmd_ = wx_cmd_ / vx_cmd_;
    }
    else
    {
      curv_cmd_ = 0.0;  // set to 0 if vx_cmd_reaches 0
    }
  }
  else {
    vx_cmd_ = 0.0;
    curv_cmd_ = 0.0;
  }
}

// get the timestamp when Phoenix sends a signal to stop ROS
void DeepOrangeStateSupervisor::getStopRos(const std_msgs::Bool::ConstPtr &msg) {
  // set the stop_ros_ variable to the value from the message
  stop_ros_ = msg->data;

  // if asserted, start a timer
  if (stop_ros_) {
    ROS_DEBUG("stop_ros signal received, starting timer");
    stop_ros_timer_.start();
  }
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
  else {
    resetMissionStatusBools();
  }
}

// sets the stack_fault_ flag if some time has passed without new command messages
void DeepOrangeStateSupervisor::assertStackFault(const ros::TimerEvent& event) {
  ROS_DEBUG("No command message received in %1.2f sec, stack fault detected", cmd_recv_timeout_s_);
  stack_fault_ = true;
}

// reset the stop_ros_ flag after some time has passed
void DeepOrangeStateSupervisor::resetStopRos(const ros::TimerEvent& event) {
  ROS_DEBUG("stop_ros timer timed out, resetting stop_ros flag");
  stop_ros_ = false;
}

// update the mission status booleans (for completed, aborted, and running statuses)
// Phoenix only uses 6 of the 10 available statuses, given in the GoalStatus message'
// the booleans can only be set to TRUE when in AU_3 or AU_4 (vehicle is operating in
// autonomy mode) to protect against a stale status being considered in the state machine
void DeepOrangeStateSupervisor::updateMissionStatusBools() {
  if (au_state_ == AU_3_READY_FOR_MISSION || au_state_ == AU_4_MISSION_IN_PROGRESS) {
    switch (mission_status_) {
      case goal_status_dummy_.SUCCEEDED: {
        resetMissionStatusBools();
        mission_completed_ = true;
        break;
      }
      case goal_status_dummy_.ACTIVE: {
        resetMissionStatusBools();
        mission_running_ = true && !stop_ros_;  // the mission should stop if stop_ros signal is received
        mission_aborted_ = false || stop_ros_;
        break;
      }
      case goal_status_dummy_.ABORTED: {
        resetMissionStatusBools();
        mission_aborted_ = true;
      }
      case goal_status_dummy_.PENDING:
      case goal_status_dummy_.PREEMPTED:
      case goal_status_dummy_.REJECTED: {
        resetMissionStatusBools();
        break;
      }
      default: {
        ROS_WARN("[updateMissionStatusBool]: Unhandled mission status %d, resetting booleans to false",
          mission_status_);

        resetMissionStatusBools();
        break;
      }
    }
  }
  else {
    resetMissionStatusBools();
  }
}

// reset the mission status booleans (to false)
void DeepOrangeStateSupervisor::resetMissionStatusBools() {
  mission_running_ = false;
  mission_completed_ = false;
  mission_aborted_ = false;
}

// timer callback to publish commands regularly
// this runs independent of receiving measurements, as the stack does not use those measurements
// in calculating the commands
void DeepOrangeStateSupervisor::updateControlCommands(const ros::TimerEvent &event) {
  updateStateMachine();

  pubCommands();
  pubDebugMsg();
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

      if (dbw_state_ >= DBW_1_WAITING_HEARTBEAT) {
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

        // initial attempt to prevent transitioning to state 3 if a mission is already running
        updateMissionStatusBools();  // need to check the latest mission status booleans
        if (mission_running_) {
          au_state_ = AU_2_WAITING_HANDOFF;  // don't transition if a mission is running
          ROS_WARN_THROTTLE(0.5, "[AU_2_WAITING_HANDOFF]: Mission is currently running.  For safety, please stop the "
            "mission before enacting the deadman switch on the vehicle controller.");
        }
        else if (stack_fault_) {
          au_state_ = AU_2_WAITING_HANDOFF;  // don't transition if Phoenix is not running
          ROS_WARN_THROTTLE(0.5, "[AU_2_WAITING_HANDOFF]: No communication with Phoenix stack.  Make sure the stack "
            "is running and relaunch if it is is not, then try enacting the deadman switch again.");
        }
        else {
          ROS_INFO("[AU_2_WAITING_HANDOFF]: Transitioning to AU_3_READY_FOR_MISSION");
        }
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_2_WAITING_HANDOFF]: Waiting for Raptor to be ready for mission");
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
      else if (stack_fault_ || dbw_state_ != DBW_3_READY_TO_DRIVE) {
        // go to state 2 when Phoenix stack has crashed, or Raptor is no longer ready to receive commands
        au_state_ = AU_2_WAITING_HANDOFF;

        if (stack_fault_) {
          ROS_WARN("[AU_3_READY_FOR_MISSION]: Lost communication with Phoenix");
        }
        else {
          ROS_WARN("[AU_3_READY_FOR_MISSION]: Raptor no longer ready for mission");
        }
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_3_READY_FOR_MISSION]: Waiting for mission");
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
      else if (mission_completed_ || mission_aborted_ || stack_fault_ || dbw_state_ <= DBW_2_WAITING_DRIVE_REQ) {
        // go to state 2 when mission has ended (or been stopped), Phoenix stack has crashed, or Raptor is no
        // longer ready to receive mission commands
        au_state_ = AU_2_WAITING_HANDOFF;

        if (stack_fault_) {
          ROS_WARN("[AU_4_MISSION_IN_PROGRESS]: Lost communication with Phoenix");
        }
        else if (mission_completed_ || mission_aborted_) {
          ROS_INFO("[AU_4_MISSION_IN_PROGRESS]: Mission complete or aborted, transitioning to AU_2_WAITING_HANDOFF");
        }
        else {
          ROS_WARN("[AU_4_MISSION_IN_PROGRESS]: Raptor no longer ready for mission");
        }
      }
      else {
        // do nothing, stay in same state
        ROS_DEBUG("[AU_4_MISSION_IN_PROGRESS]: Mission executing");
      }

      prev_au_state_ = AU_4_MISSION_IN_PROGRESS;
      break;
    }
    default: {
      au_state_ = AU_0_NO_HEARTBEAT;
      ROS_ERROR("Unknown state, state machine resetting");
      break;
    }
  }
}
}  // namespace deeporange14

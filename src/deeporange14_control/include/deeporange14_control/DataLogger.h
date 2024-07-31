/* 
A class to automatically initiate Rosbag record for GPS
data logging. The rosbag record begins when the Raptor state is SS_2
and the rosbag record is stopped when system state becomes SS_31 or
an error state is reached
*/

#ifndef _DATA_LOGGER_H_
#define _DATA_LOGGER_H_

#include <string>

#include <ros/ros.h>

#include "boost/date_time/c_local_time_adjustor.hpp"
#include "boost/date_time/local_time_adjustor.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

#include <deeporange14_msgs/RaptorStateMsg.h>

namespace deeporange14
{
class DataLogger
{
 public:
  DataLogger(ros::NodeHandle &node, ros::NodeHandle &priv_nh);
  ~DataLogger();

 private:
  void recordRosbagAndCANlog(const deeporange14_msgs::RaptorStateMsg::ConstPtr& msg);
  void monitorFileSize(std::string &can_file, std::string &ros_bag);

  ros::Timer timer_;

  std::string pid_candump;

  // Subscriber object
  ros::Subscriber sub_raptor_;
  int kill_timer;
  int logging_counter;
  double can_log_size;
  double ros_bag_size;
  double curr_can_log_size;
  double curr_ros_bag_size;
  std::string can_log_name;
  std::string ros_bag_name;

  // Recording status (currently recording / not recording)
  bool isRecording;

  std::string topic_ns = "/deeporange1314";
};
}  // namespace deeporange14

#endif  // _DATA_LOGGER_H_

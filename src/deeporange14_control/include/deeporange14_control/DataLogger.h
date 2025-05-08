/* 
A class to automatically initiate Rosbag record for GPS
data logging. The rosbag record begins when the Raptor state is SS_2
and the rosbag record is stopped when system state becomes SS_31 or
an error state is reached
*/

#ifndef DEEPORANGE14_CONTROL__DATALOGGER_H_
#define DEEPORANGE14_CONTROL__DATALOGGER_H_

#include <string>

#include <rclcpp/rclcpp.hpp>

#include "boost/date_time/c_local_time_adjustor.hpp"
#include "boost/date_time/local_time_adjustor.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

#include <deeporange14_msgs/msg/raptor_state.hpp>

namespace deeporange14
{
class DataLogger
{
public:
  DataLogger(rclcpp::Node::SharedPtr node);
  ~DataLogger();

private:
  void recordRosbagAndCANlog(const deeporange14_msgs::msg::RaptorState & msg);
  void monitorFileSize(const std::string & can_file, const std::string & ros_bag);

  rclcpp::Node::SharedPtr node_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string pid_candump;

  // Subscriber object
  rclcpp::Subscription<deeporange14_msgs::msg::RaptorState>::SharedPtr sub_raptor_;
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

#endif  // DEEPORANGE14_CONTROL__DATALOGGER_H_

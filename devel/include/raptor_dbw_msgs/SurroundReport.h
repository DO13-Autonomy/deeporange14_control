// Generated by gencpp from file raptor_dbw_msgs/SurroundReport.msg
// DO NOT EDIT!


#ifndef RAPTOR_DBW_MSGS_MESSAGE_SURROUNDREPORT_H
#define RAPTOR_DBW_MSGS_MESSAGE_SURROUNDREPORT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <raptor_dbw_msgs/SonarArcNum.h>
#include <raptor_dbw_msgs/SonarArcNum.h>
#include <raptor_dbw_msgs/SonarArcNum.h>
#include <raptor_dbw_msgs/SonarArcNum.h>
#include <raptor_dbw_msgs/SonarArcNum.h>
#include <raptor_dbw_msgs/SonarArcNum.h>

namespace raptor_dbw_msgs
{
template <class ContainerAllocator>
struct SurroundReport_
{
  typedef SurroundReport_<ContainerAllocator> Type;

  SurroundReport_()
    : header()
    , front_radar_object_distance(0.0)
    , rear_radar_object_distance(0.0)
    , rear_right()
    , rear_left()
    , rear_center()
    , front_right()
    , front_left()
    , front_center()
    , front_radar_distance_valid(false)
    , parking_sonar_data_valid(false)  {
    }
  SurroundReport_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , front_radar_object_distance(0.0)
    , rear_radar_object_distance(0.0)
    , rear_right(_alloc)
    , rear_left(_alloc)
    , rear_center(_alloc)
    , front_right(_alloc)
    , front_left(_alloc)
    , front_center(_alloc)
    , front_radar_distance_valid(false)
    , parking_sonar_data_valid(false)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef float _front_radar_object_distance_type;
  _front_radar_object_distance_type front_radar_object_distance;

   typedef float _rear_radar_object_distance_type;
  _rear_radar_object_distance_type rear_radar_object_distance;

   typedef  ::raptor_dbw_msgs::SonarArcNum_<ContainerAllocator>  _rear_right_type;
  _rear_right_type rear_right;

   typedef  ::raptor_dbw_msgs::SonarArcNum_<ContainerAllocator>  _rear_left_type;
  _rear_left_type rear_left;

   typedef  ::raptor_dbw_msgs::SonarArcNum_<ContainerAllocator>  _rear_center_type;
  _rear_center_type rear_center;

   typedef  ::raptor_dbw_msgs::SonarArcNum_<ContainerAllocator>  _front_right_type;
  _front_right_type front_right;

   typedef  ::raptor_dbw_msgs::SonarArcNum_<ContainerAllocator>  _front_left_type;
  _front_left_type front_left;

   typedef  ::raptor_dbw_msgs::SonarArcNum_<ContainerAllocator>  _front_center_type;
  _front_center_type front_center;

   typedef uint8_t _front_radar_distance_valid_type;
  _front_radar_distance_valid_type front_radar_distance_valid;

   typedef uint8_t _parking_sonar_data_valid_type;
  _parking_sonar_data_valid_type parking_sonar_data_valid;





  typedef boost::shared_ptr< ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator> const> ConstPtr;

}; // struct SurroundReport_

typedef ::raptor_dbw_msgs::SurroundReport_<std::allocator<void> > SurroundReport;

typedef boost::shared_ptr< ::raptor_dbw_msgs::SurroundReport > SurroundReportPtr;
typedef boost::shared_ptr< ::raptor_dbw_msgs::SurroundReport const> SurroundReportConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator1> & lhs, const ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.front_radar_object_distance == rhs.front_radar_object_distance &&
    lhs.rear_radar_object_distance == rhs.rear_radar_object_distance &&
    lhs.rear_right == rhs.rear_right &&
    lhs.rear_left == rhs.rear_left &&
    lhs.rear_center == rhs.rear_center &&
    lhs.front_right == rhs.front_right &&
    lhs.front_left == rhs.front_left &&
    lhs.front_center == rhs.front_center &&
    lhs.front_radar_distance_valid == rhs.front_radar_distance_valid &&
    lhs.parking_sonar_data_valid == rhs.parking_sonar_data_valid;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator1> & lhs, const ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace raptor_dbw_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7fb6baa192c10e400b9071053bd9c683";
  }

  static const char* value(const ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7fb6baa192c10e40ULL;
  static const uint64_t static_value2 = 0x0b9071053bd9c683ULL;
};

template<class ContainerAllocator>
struct DataType< ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator> >
{
  static const char* value()
  {
    return "raptor_dbw_msgs/SurroundReport";
  }

  static const char* value(const ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"float32 front_radar_object_distance # meters\n"
"float32 rear_radar_object_distance # meters\n"
"\n"
"SonarArcNum rear_right\n"
"SonarArcNum rear_left\n"
"SonarArcNum rear_center\n"
"SonarArcNum front_right\n"
"SonarArcNum front_left\n"
"SonarArcNum front_center\n"
"\n"
"bool front_radar_distance_valid\n"
"bool parking_sonar_data_valid\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: raptor_dbw_msgs/SonarArcNum\n"
"uint8 status\n"
"\n"
"# Unitless - based on bars on display\n"
"uint8 NO_DISTANCE = 0\n"
"uint8 ARC1 = 1\n"
"uint8 ARC2 = 2\n"
"uint8 ARC3 = 3\n"
"uint8 ARC4 = 4\n"
"uint8 ARC5 = 5\n"
"uint8 ARC6 = 6\n"
"uint8 ARC7 = 7\n"
"uint8 ARC8 = 8\n"
"uint8 NO_ARC = 15\n"
;
  }

  static const char* value(const ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.front_radar_object_distance);
      stream.next(m.rear_radar_object_distance);
      stream.next(m.rear_right);
      stream.next(m.rear_left);
      stream.next(m.rear_center);
      stream.next(m.front_right);
      stream.next(m.front_left);
      stream.next(m.front_center);
      stream.next(m.front_radar_distance_valid);
      stream.next(m.parking_sonar_data_valid);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SurroundReport_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::raptor_dbw_msgs::SurroundReport_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "front_radar_object_distance: ";
    Printer<float>::stream(s, indent + "  ", v.front_radar_object_distance);
    s << indent << "rear_radar_object_distance: ";
    Printer<float>::stream(s, indent + "  ", v.rear_radar_object_distance);
    s << indent << "rear_right: ";
    s << std::endl;
    Printer< ::raptor_dbw_msgs::SonarArcNum_<ContainerAllocator> >::stream(s, indent + "  ", v.rear_right);
    s << indent << "rear_left: ";
    s << std::endl;
    Printer< ::raptor_dbw_msgs::SonarArcNum_<ContainerAllocator> >::stream(s, indent + "  ", v.rear_left);
    s << indent << "rear_center: ";
    s << std::endl;
    Printer< ::raptor_dbw_msgs::SonarArcNum_<ContainerAllocator> >::stream(s, indent + "  ", v.rear_center);
    s << indent << "front_right: ";
    s << std::endl;
    Printer< ::raptor_dbw_msgs::SonarArcNum_<ContainerAllocator> >::stream(s, indent + "  ", v.front_right);
    s << indent << "front_left: ";
    s << std::endl;
    Printer< ::raptor_dbw_msgs::SonarArcNum_<ContainerAllocator> >::stream(s, indent + "  ", v.front_left);
    s << indent << "front_center: ";
    s << std::endl;
    Printer< ::raptor_dbw_msgs::SonarArcNum_<ContainerAllocator> >::stream(s, indent + "  ", v.front_center);
    s << indent << "front_radar_distance_valid: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.front_radar_distance_valid);
    s << indent << "parking_sonar_data_valid: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.parking_sonar_data_valid);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RAPTOR_DBW_MSGS_MESSAGE_SURROUNDREPORT_H

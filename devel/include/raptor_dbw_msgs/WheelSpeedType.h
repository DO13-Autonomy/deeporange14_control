// Generated by gencpp from file raptor_dbw_msgs/WheelSpeedType.msg
// DO NOT EDIT!


#ifndef RAPTOR_DBW_MSGS_MESSAGE_WHEELSPEEDTYPE_H
#define RAPTOR_DBW_MSGS_MESSAGE_WHEELSPEEDTYPE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace raptor_dbw_msgs
{
template <class ContainerAllocator>
struct WheelSpeedType_
{
  typedef WheelSpeedType_<ContainerAllocator> Type;

  WheelSpeedType_()
    : value(0)  {
    }
  WheelSpeedType_(const ContainerAllocator& _alloc)
    : value(0)  {
  (void)_alloc;
    }



   typedef uint8_t _value_type;
  _value_type value;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(ANGULAR)
  #undef ANGULAR
#endif
#if defined(_WIN32) && defined(LINEAR)
  #undef LINEAR
#endif

  enum {
    ANGULAR = 0u,
    LINEAR = 1u,
  };


  typedef boost::shared_ptr< ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator> const> ConstPtr;

}; // struct WheelSpeedType_

typedef ::raptor_dbw_msgs::WheelSpeedType_<std::allocator<void> > WheelSpeedType;

typedef boost::shared_ptr< ::raptor_dbw_msgs::WheelSpeedType > WheelSpeedTypePtr;
typedef boost::shared_ptr< ::raptor_dbw_msgs::WheelSpeedType const> WheelSpeedTypeConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator1> & lhs, const ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator2> & rhs)
{
  return lhs.value == rhs.value;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator1> & lhs, const ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace raptor_dbw_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3449744e3a452eb36b1db88cb52b05b8";
  }

  static const char* value(const ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3449744e3a452eb3ULL;
  static const uint64_t static_value2 = 0x6b1db88cb52b05b8ULL;
};

template<class ContainerAllocator>
struct DataType< ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "raptor_dbw_msgs/WheelSpeedType";
  }

  static const char* value(const ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 value\n"
"\n"
"uint8 ANGULAR = 0\n"
"uint8 LINEAR = 1\n"
;
  }

  static const char* value(const ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct WheelSpeedType_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::raptor_dbw_msgs::WheelSpeedType_<ContainerAllocator>& v)
  {
    s << indent << "value: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RAPTOR_DBW_MSGS_MESSAGE_WHEELSPEEDTYPE_H

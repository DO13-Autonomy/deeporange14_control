// Generated by gencpp from file raptor_dbw_msgs/Gear.msg
// DO NOT EDIT!


#ifndef RAPTOR_DBW_MSGS_MESSAGE_GEAR_H
#define RAPTOR_DBW_MSGS_MESSAGE_GEAR_H


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
struct Gear_
{
  typedef Gear_<ContainerAllocator> Type;

  Gear_()
    : gear(0)  {
    }
  Gear_(const ContainerAllocator& _alloc)
    : gear(0)  {
  (void)_alloc;
    }



   typedef uint8_t _gear_type;
  _gear_type gear;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(NONE)
  #undef NONE
#endif
#if defined(_WIN32) && defined(PARK)
  #undef PARK
#endif
#if defined(_WIN32) && defined(REVERSE)
  #undef REVERSE
#endif
#if defined(_WIN32) && defined(NEUTRAL)
  #undef NEUTRAL
#endif
#if defined(_WIN32) && defined(DRIVE)
  #undef DRIVE
#endif
#if defined(_WIN32) && defined(LOW)
  #undef LOW
#endif

  enum {
    NONE = 0u,
    PARK = 1u,
    REVERSE = 2u,
    NEUTRAL = 3u,
    DRIVE = 4u,
    LOW = 5u,
  };


  typedef boost::shared_ptr< ::raptor_dbw_msgs::Gear_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::raptor_dbw_msgs::Gear_<ContainerAllocator> const> ConstPtr;

}; // struct Gear_

typedef ::raptor_dbw_msgs::Gear_<std::allocator<void> > Gear;

typedef boost::shared_ptr< ::raptor_dbw_msgs::Gear > GearPtr;
typedef boost::shared_ptr< ::raptor_dbw_msgs::Gear const> GearConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::raptor_dbw_msgs::Gear_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::raptor_dbw_msgs::Gear_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::raptor_dbw_msgs::Gear_<ContainerAllocator1> & lhs, const ::raptor_dbw_msgs::Gear_<ContainerAllocator2> & rhs)
{
  return lhs.gear == rhs.gear;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::raptor_dbw_msgs::Gear_<ContainerAllocator1> & lhs, const ::raptor_dbw_msgs::Gear_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace raptor_dbw_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::raptor_dbw_msgs::Gear_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::raptor_dbw_msgs::Gear_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::raptor_dbw_msgs::Gear_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::raptor_dbw_msgs::Gear_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::raptor_dbw_msgs::Gear_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::raptor_dbw_msgs::Gear_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::raptor_dbw_msgs::Gear_<ContainerAllocator> >
{
  static const char* value()
  {
    return "79b3cd667a7556f4bc4a66af7d189c96";
  }

  static const char* value(const ::raptor_dbw_msgs::Gear_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x79b3cd667a7556f4ULL;
  static const uint64_t static_value2 = 0xbc4a66af7d189c96ULL;
};

template<class ContainerAllocator>
struct DataType< ::raptor_dbw_msgs::Gear_<ContainerAllocator> >
{
  static const char* value()
  {
    return "raptor_dbw_msgs/Gear";
  }

  static const char* value(const ::raptor_dbw_msgs::Gear_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::raptor_dbw_msgs::Gear_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 gear\n"
"\n"
"uint8 NONE=0\n"
"uint8 PARK=1\n"
"uint8 REVERSE=2\n"
"uint8 NEUTRAL=3\n"
"uint8 DRIVE=4\n"
"uint8 LOW=5\n"
;
  }

  static const char* value(const ::raptor_dbw_msgs::Gear_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::raptor_dbw_msgs::Gear_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.gear);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Gear_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::raptor_dbw_msgs::Gear_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::raptor_dbw_msgs::Gear_<ContainerAllocator>& v)
  {
    s << indent << "gear: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gear);
  }
};

} // namespace message_operations
} // namespace ros

#endif // RAPTOR_DBW_MSGS_MESSAGE_GEAR_H

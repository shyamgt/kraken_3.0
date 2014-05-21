/* Auto-generated by genmsg_cpp for file /home/prudhvi/ros_ws/kraken_msgs/msg/orientResult.msg */
#ifndef KRAKEN_MSGS_MESSAGE_ORIENTRESULT_H
#define KRAKEN_MSGS_MESSAGE_ORIENTRESULT_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"


namespace kraken_msgs
{
template <class ContainerAllocator>
struct orientResult_ {
  typedef orientResult_<ContainerAllocator> Type;

  orientResult_()
  : roll_f(0.0)
  , pitch_f(0.0)
  , yaw_f(0.0)
  {
  }

  orientResult_(const ContainerAllocator& _alloc)
  : roll_f(0.0)
  , pitch_f(0.0)
  , yaw_f(0.0)
  {
  }

  typedef float _roll_f_type;
  float roll_f;

  typedef float _pitch_f_type;
  float pitch_f;

  typedef float _yaw_f_type;
  float yaw_f;


  typedef boost::shared_ptr< ::kraken_msgs::orientResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kraken_msgs::orientResult_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct orientResult
typedef  ::kraken_msgs::orientResult_<std::allocator<void> > orientResult;

typedef boost::shared_ptr< ::kraken_msgs::orientResult> orientResultPtr;
typedef boost::shared_ptr< ::kraken_msgs::orientResult const> orientResultConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::kraken_msgs::orientResult_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::kraken_msgs::orientResult_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace kraken_msgs

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::kraken_msgs::orientResult_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::kraken_msgs::orientResult_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::kraken_msgs::orientResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "517234e8df6e71015d90db115635924b";
  }

  static const char* value(const  ::kraken_msgs::orientResult_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x517234e8df6e7101ULL;
  static const uint64_t static_value2 = 0x5d90db115635924bULL;
};

template<class ContainerAllocator>
struct DataType< ::kraken_msgs::orientResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "kraken_msgs/orientResult";
  }

  static const char* value(const  ::kraken_msgs::orientResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::kraken_msgs::orientResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#result\n\
float32 roll_f\n\
float32 pitch_f\n\
float32 yaw_f\n\
\n\
";
  }

  static const char* value(const  ::kraken_msgs::orientResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::kraken_msgs::orientResult_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::kraken_msgs::orientResult_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.roll_f);
    stream.next(m.pitch_f);
    stream.next(m.yaw_f);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct orientResult_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kraken_msgs::orientResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::kraken_msgs::orientResult_<ContainerAllocator> & v) 
  {
    s << indent << "roll_f: ";
    Printer<float>::stream(s, indent + "  ", v.roll_f);
    s << indent << "pitch_f: ";
    Printer<float>::stream(s, indent + "  ", v.pitch_f);
    s << indent << "yaw_f: ";
    Printer<float>::stream(s, indent + "  ", v.yaw_f);
  }
};


} // namespace message_operations
} // namespace ros

#endif // KRAKEN_MSGS_MESSAGE_ORIENTRESULT_H

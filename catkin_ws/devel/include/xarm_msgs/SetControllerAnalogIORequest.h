// Generated by gencpp from file xarm_msgs/SetControllerAnalogIORequest.msg
// DO NOT EDIT!


#ifndef XARM_MSGS_MESSAGE_SETCONTROLLERANALOGIOREQUEST_H
#define XARM_MSGS_MESSAGE_SETCONTROLLERANALOGIOREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace xarm_msgs
{
template <class ContainerAllocator>
struct SetControllerAnalogIORequest_
{
  typedef SetControllerAnalogIORequest_<ContainerAllocator> Type;

  SetControllerAnalogIORequest_()
    : port_num(0)
    , analog_value(0.0)  {
    }
  SetControllerAnalogIORequest_(const ContainerAllocator& _alloc)
    : port_num(0)
    , analog_value(0.0)  {
  (void)_alloc;
    }



   typedef int16_t _port_num_type;
  _port_num_type port_num;

   typedef float _analog_value_type;
  _analog_value_type analog_value;





  typedef boost::shared_ptr< ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator> const> ConstPtr;

}; // struct SetControllerAnalogIORequest_

typedef ::xarm_msgs::SetControllerAnalogIORequest_<std::allocator<void> > SetControllerAnalogIORequest;

typedef boost::shared_ptr< ::xarm_msgs::SetControllerAnalogIORequest > SetControllerAnalogIORequestPtr;
typedef boost::shared_ptr< ::xarm_msgs::SetControllerAnalogIORequest const> SetControllerAnalogIORequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator1> & lhs, const ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator2> & rhs)
{
  return lhs.port_num == rhs.port_num &&
    lhs.analog_value == rhs.analog_value;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator1> & lhs, const ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace xarm_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f4df395f9657a7248b13463d3cf4caac";
  }

  static const char* value(const ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf4df395f9657a724ULL;
  static const uint64_t static_value2 = 0x8b13463d3cf4caacULL;
};

template<class ContainerAllocator>
struct DataType< ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "xarm_msgs/SetControllerAnalogIORequest";
  }

  static const char* value(const ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Setting the analog Output port value at robot controller, io_num: from 1 to 2\n"
"\n"
"int16 port_num\n"
"\n"
"float32 analog_value\n"
"\n"
;
  }

  static const char* value(const ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.port_num);
      stream.next(m.analog_value);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetControllerAnalogIORequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::xarm_msgs::SetControllerAnalogIORequest_<ContainerAllocator>& v)
  {
    s << indent << "port_num: ";
    Printer<int16_t>::stream(s, indent + "  ", v.port_num);
    s << indent << "analog_value: ";
    Printer<float>::stream(s, indent + "  ", v.analog_value);
  }
};

} // namespace message_operations
} // namespace ros

#endif // XARM_MSGS_MESSAGE_SETCONTROLLERANALOGIOREQUEST_H

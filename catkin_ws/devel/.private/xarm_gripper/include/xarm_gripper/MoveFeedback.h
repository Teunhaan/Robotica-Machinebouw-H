// Generated by gencpp from file xarm_gripper/MoveFeedback.msg
// DO NOT EDIT!


#ifndef XARM_GRIPPER_MESSAGE_MOVEFEEDBACK_H
#define XARM_GRIPPER_MESSAGE_MOVEFEEDBACK_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace xarm_gripper
{
template <class ContainerAllocator>
struct MoveFeedback_
{
  typedef MoveFeedback_<ContainerAllocator> Type;

  MoveFeedback_()
    : current_pulse(0.0)  {
    }
  MoveFeedback_(const ContainerAllocator& _alloc)
    : current_pulse(0.0)  {
  (void)_alloc;
    }



   typedef float _current_pulse_type;
  _current_pulse_type current_pulse;





  typedef boost::shared_ptr< ::xarm_gripper::MoveFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::xarm_gripper::MoveFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct MoveFeedback_

typedef ::xarm_gripper::MoveFeedback_<std::allocator<void> > MoveFeedback;

typedef boost::shared_ptr< ::xarm_gripper::MoveFeedback > MoveFeedbackPtr;
typedef boost::shared_ptr< ::xarm_gripper::MoveFeedback const> MoveFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::xarm_gripper::MoveFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::xarm_gripper::MoveFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::xarm_gripper::MoveFeedback_<ContainerAllocator1> & lhs, const ::xarm_gripper::MoveFeedback_<ContainerAllocator2> & rhs)
{
  return lhs.current_pulse == rhs.current_pulse;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::xarm_gripper::MoveFeedback_<ContainerAllocator1> & lhs, const ::xarm_gripper::MoveFeedback_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace xarm_gripper

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::xarm_gripper::MoveFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::xarm_gripper::MoveFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xarm_gripper::MoveFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xarm_gripper::MoveFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xarm_gripper::MoveFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xarm_gripper::MoveFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::xarm_gripper::MoveFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a244e071efec9946f249ffbbf7ee56b0";
  }

  static const char* value(const ::xarm_gripper::MoveFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa244e071efec9946ULL;
  static const uint64_t static_value2 = 0xf249ffbbf7ee56b0ULL;
};

template<class ContainerAllocator>
struct DataType< ::xarm_gripper::MoveFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "xarm_gripper/MoveFeedback";
  }

  static const char* value(const ::xarm_gripper::MoveFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::xarm_gripper::MoveFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n"
"\n"
"float32 current_pulse\n"
"\n"
;
  }

  static const char* value(const ::xarm_gripper::MoveFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::xarm_gripper::MoveFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.current_pulse);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MoveFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::xarm_gripper::MoveFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::xarm_gripper::MoveFeedback_<ContainerAllocator>& v)
  {
    s << indent << "current_pulse: ";
    Printer<float>::stream(s, indent + "  ", v.current_pulse);
  }
};

} // namespace message_operations
} // namespace ros

#endif // XARM_GRIPPER_MESSAGE_MOVEFEEDBACK_H

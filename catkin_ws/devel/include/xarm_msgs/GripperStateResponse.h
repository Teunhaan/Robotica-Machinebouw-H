// Generated by gencpp from file xarm_msgs/GripperStateResponse.msg
// DO NOT EDIT!


#ifndef XARM_MSGS_MESSAGE_GRIPPERSTATERESPONSE_H
#define XARM_MSGS_MESSAGE_GRIPPERSTATERESPONSE_H


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
struct GripperStateResponse_
{
  typedef GripperStateResponse_<ContainerAllocator> Type;

  GripperStateResponse_()
    : ret(0)
    , message()
    , curr_pos(0.0)
    , err_code(0)  {
    }
  GripperStateResponse_(const ContainerAllocator& _alloc)
    : ret(0)
    , message(_alloc)
    , curr_pos(0.0)
    , err_code(0)  {
  (void)_alloc;
    }



   typedef int16_t _ret_type;
  _ret_type ret;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _message_type;
  _message_type message;

   typedef float _curr_pos_type;
  _curr_pos_type curr_pos;

   typedef int16_t _err_code_type;
  _err_code_type err_code;





  typedef boost::shared_ptr< ::xarm_msgs::GripperStateResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::xarm_msgs::GripperStateResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GripperStateResponse_

typedef ::xarm_msgs::GripperStateResponse_<std::allocator<void> > GripperStateResponse;

typedef boost::shared_ptr< ::xarm_msgs::GripperStateResponse > GripperStateResponsePtr;
typedef boost::shared_ptr< ::xarm_msgs::GripperStateResponse const> GripperStateResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::xarm_msgs::GripperStateResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::xarm_msgs::GripperStateResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::xarm_msgs::GripperStateResponse_<ContainerAllocator1> & lhs, const ::xarm_msgs::GripperStateResponse_<ContainerAllocator2> & rhs)
{
  return lhs.ret == rhs.ret &&
    lhs.message == rhs.message &&
    lhs.curr_pos == rhs.curr_pos &&
    lhs.err_code == rhs.err_code;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::xarm_msgs::GripperStateResponse_<ContainerAllocator1> & lhs, const ::xarm_msgs::GripperStateResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace xarm_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::xarm_msgs::GripperStateResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::xarm_msgs::GripperStateResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xarm_msgs::GripperStateResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::xarm_msgs::GripperStateResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xarm_msgs::GripperStateResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::xarm_msgs::GripperStateResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::xarm_msgs::GripperStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b5eb0261d03e545bc9905bb8e7e041a8";
  }

  static const char* value(const ::xarm_msgs::GripperStateResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb5eb0261d03e545bULL;
  static const uint64_t static_value2 = 0xc9905bb8e7e041a8ULL;
};

template<class ContainerAllocator>
struct DataType< ::xarm_msgs::GripperStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "xarm_msgs/GripperStateResponse";
  }

  static const char* value(const ::xarm_msgs::GripperStateResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::xarm_msgs::GripperStateResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 ret\n"
"string message\n"
"float32 curr_pos\n"
"int16 err_code\n"
;
  }

  static const char* value(const ::xarm_msgs::GripperStateResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::xarm_msgs::GripperStateResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.ret);
      stream.next(m.message);
      stream.next(m.curr_pos);
      stream.next(m.err_code);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GripperStateResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::xarm_msgs::GripperStateResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::xarm_msgs::GripperStateResponse_<ContainerAllocator>& v)
  {
    s << indent << "ret: ";
    Printer<int16_t>::stream(s, indent + "  ", v.ret);
    s << indent << "message: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.message);
    s << indent << "curr_pos: ";
    Printer<float>::stream(s, indent + "  ", v.curr_pos);
    s << indent << "err_code: ";
    Printer<int16_t>::stream(s, indent + "  ", v.err_code);
  }
};

} // namespace message_operations
} // namespace ros

#endif // XARM_MSGS_MESSAGE_GRIPPERSTATERESPONSE_H

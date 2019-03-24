// Generated by gencpp from file dji_sdk/CameraActionControlRequest.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_CAMERAACTIONCONTROLREQUEST_H
#define DJI_SDK_MESSAGE_CAMERAACTIONCONTROLREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dji_sdk
{
template <class ContainerAllocator>
struct CameraActionControlRequest_
{
  typedef CameraActionControlRequest_<ContainerAllocator> Type;

  CameraActionControlRequest_()
    : camera_action(0)  {
    }
  CameraActionControlRequest_(const ContainerAllocator& _alloc)
    : camera_action(0)  {
    }



   typedef uint8_t _camera_action_type;
  _camera_action_type camera_action;




  typedef boost::shared_ptr< ::dji_sdk::CameraActionControlRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::CameraActionControlRequest_<ContainerAllocator> const> ConstPtr;

}; // struct CameraActionControlRequest_

typedef ::dji_sdk::CameraActionControlRequest_<std::allocator<void> > CameraActionControlRequest;

typedef boost::shared_ptr< ::dji_sdk::CameraActionControlRequest > CameraActionControlRequestPtr;
typedef boost::shared_ptr< ::dji_sdk::CameraActionControlRequest const> CameraActionControlRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::CameraActionControlRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::CameraActionControlRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace dji_sdk

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'nav_msgs': ['/opt/ros/indigo/share/nav_msgs/cmake/../msg'], 'dji_sdk': ['/home/tx/catkin_ws/src/Onboard-SDK-6.2  obstacle/dji_sdk/msg', '/home/tx/catkin_ws/devel/share/dji_sdk/msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::CameraActionControlRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::CameraActionControlRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::CameraActionControlRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::CameraActionControlRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::CameraActionControlRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::CameraActionControlRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::CameraActionControlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a4ed1822b47772deebd56bdca0095874";
  }

  static const char* value(const ::dji_sdk::CameraActionControlRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa4ed1822b47772deULL;
  static const uint64_t static_value2 = 0xebd56bdca0095874ULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::CameraActionControlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/CameraActionControlRequest";
  }

  static const char* value(const ::dji_sdk::CameraActionControlRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::CameraActionControlRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
\n\
\n\
uint8 camera_action\n\
";
  }

  static const char* value(const ::dji_sdk::CameraActionControlRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::CameraActionControlRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.camera_action);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct CameraActionControlRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::CameraActionControlRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::CameraActionControlRequest_<ContainerAllocator>& v)
  {
    s << indent << "camera_action: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.camera_action);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_CAMERAACTIONCONTROLREQUEST_H

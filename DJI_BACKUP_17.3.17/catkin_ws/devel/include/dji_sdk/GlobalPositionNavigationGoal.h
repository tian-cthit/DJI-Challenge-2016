// Generated by gencpp from file dji_sdk/GlobalPositionNavigationGoal.msg
// DO NOT EDIT!


#ifndef DJI_SDK_MESSAGE_GLOBALPOSITIONNAVIGATIONGOAL_H
#define DJI_SDK_MESSAGE_GLOBALPOSITIONNAVIGATIONGOAL_H


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
struct GlobalPositionNavigationGoal_
{
  typedef GlobalPositionNavigationGoal_<ContainerAllocator> Type;

  GlobalPositionNavigationGoal_()
    : latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)  {
    }
  GlobalPositionNavigationGoal_(const ContainerAllocator& _alloc)
    : latitude(0.0)
    , longitude(0.0)
    , altitude(0.0)  {
    }



   typedef double _latitude_type;
  _latitude_type latitude;

   typedef double _longitude_type;
  _longitude_type longitude;

   typedef float _altitude_type;
  _altitude_type altitude;




  typedef boost::shared_ptr< ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator> const> ConstPtr;

}; // struct GlobalPositionNavigationGoal_

typedef ::dji_sdk::GlobalPositionNavigationGoal_<std::allocator<void> > GlobalPositionNavigationGoal;

typedef boost::shared_ptr< ::dji_sdk::GlobalPositionNavigationGoal > GlobalPositionNavigationGoalPtr;
typedef boost::shared_ptr< ::dji_sdk::GlobalPositionNavigationGoal const> GlobalPositionNavigationGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "055ec9ba37e4c88f434ae1fcb281ad29";
  }

  static const char* value(const ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x055ec9ba37e4c88fULL;
  static const uint64_t static_value2 = 0x434ae1fcb281ad29ULL;
};

template<class ContainerAllocator>
struct DataType< ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dji_sdk/GlobalPositionNavigationGoal";
  }

  static const char* value(const ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#latitude is in degree\n\
float64 latitude\n\
#longitude is in degree\n\
float64 longitude\n\
float32 altitude\n\
";
  }

  static const char* value(const ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.latitude);
      stream.next(m.longitude);
      stream.next(m.altitude);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct GlobalPositionNavigationGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dji_sdk::GlobalPositionNavigationGoal_<ContainerAllocator>& v)
  {
    s << indent << "latitude: ";
    Printer<double>::stream(s, indent + "  ", v.latitude);
    s << indent << "longitude: ";
    Printer<double>::stream(s, indent + "  ", v.longitude);
    s << indent << "altitude: ";
    Printer<float>::stream(s, indent + "  ", v.altitude);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DJI_SDK_MESSAGE_GLOBALPOSITIONNAVIGATIONGOAL_H

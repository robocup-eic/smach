// Generated by gencpp from file cr3_moveit_control/cr3_placeResponse.msg
// DO NOT EDIT!


#ifndef CR3_MOVEIT_CONTROL_MESSAGE_CR3_PLACERESPONSE_H
#define CR3_MOVEIT_CONTROL_MESSAGE_CR3_PLACERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cr3_moveit_control
{
template <class ContainerAllocator>
struct cr3_placeResponse_
{
  typedef cr3_placeResponse_<ContainerAllocator> Type;

  cr3_placeResponse_()
    : success_place(false)  {
    }
  cr3_placeResponse_(const ContainerAllocator& _alloc)
    : success_place(false)  {
  (void)_alloc;
    }



   typedef uint8_t _success_place_type;
  _success_place_type success_place;





  typedef boost::shared_ptr< ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator> const> ConstPtr;

}; // struct cr3_placeResponse_

typedef ::cr3_moveit_control::cr3_placeResponse_<std::allocator<void> > cr3_placeResponse;

typedef boost::shared_ptr< ::cr3_moveit_control::cr3_placeResponse > cr3_placeResponsePtr;
typedef boost::shared_ptr< ::cr3_moveit_control::cr3_placeResponse const> cr3_placeResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator1> & lhs, const ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator2> & rhs)
{
  return lhs.success_place == rhs.success_place;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator1> & lhs, const ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace cr3_moveit_control

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "825ee4be91d2b6895d6516c9ae0d7978";
  }

  static const char* value(const ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x825ee4be91d2b689ULL;
  static const uint64_t static_value2 = 0x5d6516c9ae0d7978ULL;
};

template<class ContainerAllocator>
struct DataType< ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cr3_moveit_control/cr3_placeResponse";
  }

  static const char* value(const ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool success_place\n"
;
  }

  static const char* value(const ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.success_place);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct cr3_placeResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cr3_moveit_control::cr3_placeResponse_<ContainerAllocator>& v)
  {
    s << indent << "success_place: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.success_place);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CR3_MOVEIT_CONTROL_MESSAGE_CR3_PLACERESPONSE_H

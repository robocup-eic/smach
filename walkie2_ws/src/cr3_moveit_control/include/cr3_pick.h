// Generated by gencpp from file cr3_moveit_control/cr3_pick.msg
// DO NOT EDIT!


#ifndef CR3_MOVEIT_CONTROL_MESSAGE_CR3_PICK_H
#define CR3_MOVEIT_CONTROL_MESSAGE_CR3_PICK_H

#include <ros/service_traits.h>


#include <cr3_moveit_control/cr3_pickRequest.h>
#include <cr3_moveit_control/cr3_pickResponse.h>


namespace cr3_moveit_control
{

struct cr3_pick
{

typedef cr3_pickRequest Request;
typedef cr3_pickResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct cr3_pick
} // namespace cr3_moveit_control


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::cr3_moveit_control::cr3_pick > {
  static const char* value()
  {
    return "eb45ac67896f009e7ad5e11b33cc4fee";
  }

  static const char* value(const ::cr3_moveit_control::cr3_pick&) { return value(); }
};

template<>
struct DataType< ::cr3_moveit_control::cr3_pick > {
  static const char* value()
  {
    return "cr3_moveit_control/cr3_pick";
  }

  static const char* value(const ::cr3_moveit_control::cr3_pick&) { return value(); }
};


// service_traits::MD5Sum< ::cr3_moveit_control::cr3_pickRequest> should match
// service_traits::MD5Sum< ::cr3_moveit_control::cr3_pick >
template<>
struct MD5Sum< ::cr3_moveit_control::cr3_pickRequest>
{
  static const char* value()
  {
    return MD5Sum< ::cr3_moveit_control::cr3_pick >::value();
  }
  static const char* value(const ::cr3_moveit_control::cr3_pickRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::cr3_moveit_control::cr3_pickRequest> should match
// service_traits::DataType< ::cr3_moveit_control::cr3_pick >
template<>
struct DataType< ::cr3_moveit_control::cr3_pickRequest>
{
  static const char* value()
  {
    return DataType< ::cr3_moveit_control::cr3_pick >::value();
  }
  static const char* value(const ::cr3_moveit_control::cr3_pickRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::cr3_moveit_control::cr3_pickResponse> should match
// service_traits::MD5Sum< ::cr3_moveit_control::cr3_pick >
template<>
struct MD5Sum< ::cr3_moveit_control::cr3_pickResponse>
{
  static const char* value()
  {
    return MD5Sum< ::cr3_moveit_control::cr3_pick >::value();
  }
  static const char* value(const ::cr3_moveit_control::cr3_pickResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::cr3_moveit_control::cr3_pickResponse> should match
// service_traits::DataType< ::cr3_moveit_control::cr3_pick >
template<>
struct DataType< ::cr3_moveit_control::cr3_pickResponse>
{
  static const char* value()
  {
    return DataType< ::cr3_moveit_control::cr3_pick >::value();
  }
  static const char* value(const ::cr3_moveit_control::cr3_pickResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CR3_MOVEIT_CONTROL_MESSAGE_CR3_PICK_H
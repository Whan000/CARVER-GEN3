// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from amt212ev_interfaces:msg/AmtRead.idl
// generated code does not contain a copyright notice

#ifndef AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "amt212ev_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "amt212ev_interfaces/msg/detail/amt_read__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace amt212ev_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_amt212ev_interfaces
cdr_serialize(
  const amt212ev_interfaces::msg::AmtRead & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_amt212ev_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  amt212ev_interfaces::msg::AmtRead & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_amt212ev_interfaces
get_serialized_size(
  const amt212ev_interfaces::msg::AmtRead & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_amt212ev_interfaces
max_serialized_size_AmtRead(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace amt212ev_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_amt212ev_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, amt212ev_interfaces, msg, AmtRead)();

#ifdef __cplusplus
}
#endif

#endif  // AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from amt212ev_interfaces:msg/AmtRead.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "amt212ev_interfaces/msg/detail/amt_read__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace amt212ev_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void AmtRead_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) amt212ev_interfaces::msg::AmtRead(_init);
}

void AmtRead_fini_function(void * message_memory)
{
  auto typed_message = static_cast<amt212ev_interfaces::msg::AmtRead *>(message_memory);
  typed_message->~AmtRead();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember AmtRead_message_member_array[2] = {
  {
    "rads",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(amt212ev_interfaces::msg::AmtRead, rads),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "radps",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(amt212ev_interfaces::msg::AmtRead, radps),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers AmtRead_message_members = {
  "amt212ev_interfaces::msg",  // message namespace
  "AmtRead",  // message name
  2,  // number of fields
  sizeof(amt212ev_interfaces::msg::AmtRead),
  AmtRead_message_member_array,  // message members
  AmtRead_init_function,  // function to initialize message memory (memory has to be allocated)
  AmtRead_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t AmtRead_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &AmtRead_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace amt212ev_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<amt212ev_interfaces::msg::AmtRead>()
{
  return &::amt212ev_interfaces::msg::rosidl_typesupport_introspection_cpp::AmtRead_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, amt212ev_interfaces, msg, AmtRead)() {
  return &::amt212ev_interfaces::msg::rosidl_typesupport_introspection_cpp::AmtRead_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

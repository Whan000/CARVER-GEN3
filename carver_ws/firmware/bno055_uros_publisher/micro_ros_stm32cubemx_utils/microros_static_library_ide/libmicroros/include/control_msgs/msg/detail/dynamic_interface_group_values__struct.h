// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from control_msgs:msg/DynamicInterfaceGroupValues.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_GROUP_VALUES__STRUCT_H_
#define CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_GROUP_VALUES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'interface_groups'
#include "rosidl_runtime_c/string.h"
// Member 'interface_values'
#include "control_msgs/msg/detail/interface_value__struct.h"

/// Struct defined in msg/DynamicInterfaceGroupValues in the package control_msgs.
typedef struct control_msgs__msg__DynamicInterfaceGroupValues
{
  std_msgs__msg__Header header;
  /// List of interface group names , e.g. ["flange_analog_IOs", "flange_vacuum"]
  rosidl_runtime_c__String__Sequence interface_groups;
  /// Key-value pairs representing interfaces and their corresponding values for each interface group listed in `interface_groups`
  control_msgs__msg__InterfaceValue__Sequence interface_values;
} control_msgs__msg__DynamicInterfaceGroupValues;

// Struct for a sequence of control_msgs__msg__DynamicInterfaceGroupValues.
typedef struct control_msgs__msg__DynamicInterfaceGroupValues__Sequence
{
  control_msgs__msg__DynamicInterfaceGroupValues * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} control_msgs__msg__DynamicInterfaceGroupValues__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_GROUP_VALUES__STRUCT_H_

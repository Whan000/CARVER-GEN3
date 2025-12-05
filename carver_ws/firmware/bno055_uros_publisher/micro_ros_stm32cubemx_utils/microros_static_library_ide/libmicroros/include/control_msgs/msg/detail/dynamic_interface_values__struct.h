// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from control_msgs:msg/DynamicInterfaceValues.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_VALUES__STRUCT_H_
#define CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_VALUES__STRUCT_H_

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
// Member 'states'
// Member 'commands'
#include "control_msgs/msg/detail/interface_value__struct.h"

/// Struct defined in msg/DynamicInterfaceValues in the package control_msgs.
typedef struct control_msgs__msg__DynamicInterfaceValues
{
  std_msgs__msg__Header header;
  control_msgs__msg__InterfaceValue states;
  control_msgs__msg__InterfaceValue commands;
} control_msgs__msg__DynamicInterfaceValues;

// Struct for a sequence of control_msgs__msg__DynamicInterfaceValues.
typedef struct control_msgs__msg__DynamicInterfaceValues__Sequence
{
  control_msgs__msg__DynamicInterfaceValues * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} control_msgs__msg__DynamicInterfaceValues__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_MSGS__MSG__DETAIL__DYNAMIC_INTERFACE_VALUES__STRUCT_H_

// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from control_msgs:msg/SteeringControllerCommand.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__STEERING_CONTROLLER_COMMAND__STRUCT_H_
#define CONTROL_MSGS__MSG__DETAIL__STEERING_CONTROLLER_COMMAND__STRUCT_H_

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

/// Struct defined in msg/SteeringControllerCommand in the package control_msgs.
typedef struct control_msgs__msg__SteeringControllerCommand
{
  std_msgs__msg__Header header;
  /// in rad
  double steering_angle;
  /// in m/s
  double linear_velocity;
} control_msgs__msg__SteeringControllerCommand;

// Struct for a sequence of control_msgs__msg__SteeringControllerCommand.
typedef struct control_msgs__msg__SteeringControllerCommand__Sequence
{
  control_msgs__msg__SteeringControllerCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} control_msgs__msg__SteeringControllerCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_MSGS__MSG__DETAIL__STEERING_CONTROLLER_COMMAND__STRUCT_H_

// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from control_msgs:msg/SpeedScalingFactor.idl
// generated code does not contain a copyright notice

#ifndef CONTROL_MSGS__MSG__DETAIL__SPEED_SCALING_FACTOR__STRUCT_H_
#define CONTROL_MSGS__MSG__DETAIL__SPEED_SCALING_FACTOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SpeedScalingFactor in the package control_msgs.
/**
  * This message contains a scaling factor to scale trajectory execution. A factor of 1.0 means
  * execution at normal speed, a factor of 0.0 means a full pause.
  * Negative values are not allowed (Which should be checked by any instance consuming this message).
 */
typedef struct control_msgs__msg__SpeedScalingFactor
{
  double factor;
} control_msgs__msg__SpeedScalingFactor;

// Struct for a sequence of control_msgs__msg__SpeedScalingFactor.
typedef struct control_msgs__msg__SpeedScalingFactor__Sequence
{
  control_msgs__msg__SpeedScalingFactor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} control_msgs__msg__SpeedScalingFactor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CONTROL_MSGS__MSG__DETAIL__SPEED_SCALING_FACTOR__STRUCT_H_

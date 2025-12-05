// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from amt212ev_interfaces:msg/AmtRead.idl
// generated code does not contain a copyright notice

#ifndef AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__STRUCT_H_
#define AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/AmtRead in the package amt212ev_interfaces.
typedef struct amt212ev_interfaces__msg__AmtRead
{
  double rads;
  double radps;
} amt212ev_interfaces__msg__AmtRead;

// Struct for a sequence of amt212ev_interfaces__msg__AmtRead.
typedef struct amt212ev_interfaces__msg__AmtRead__Sequence
{
  amt212ev_interfaces__msg__AmtRead * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} amt212ev_interfaces__msg__AmtRead__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__STRUCT_H_

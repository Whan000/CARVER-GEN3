// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from amt212ev_interfaces:msg/AmtRead.idl
// generated code does not contain a copyright notice
#include "amt212ev_interfaces/msg/detail/amt_read__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "amt212ev_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "amt212ev_interfaces/msg/detail/amt_read__struct.h"
#include "amt212ev_interfaces/msg/detail/amt_read__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _AmtRead__ros_msg_type = amt212ev_interfaces__msg__AmtRead;

static bool _AmtRead__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _AmtRead__ros_msg_type * ros_message = static_cast<const _AmtRead__ros_msg_type *>(untyped_ros_message);
  // Field name: rads
  {
    cdr << ros_message->rads;
  }

  // Field name: radps
  {
    cdr << ros_message->radps;
  }

  return true;
}

static bool _AmtRead__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _AmtRead__ros_msg_type * ros_message = static_cast<_AmtRead__ros_msg_type *>(untyped_ros_message);
  // Field name: rads
  {
    cdr >> ros_message->rads;
  }

  // Field name: radps
  {
    cdr >> ros_message->radps;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_amt212ev_interfaces
size_t get_serialized_size_amt212ev_interfaces__msg__AmtRead(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _AmtRead__ros_msg_type * ros_message = static_cast<const _AmtRead__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name rads
  {
    size_t item_size = sizeof(ros_message->rads);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name radps
  {
    size_t item_size = sizeof(ros_message->radps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _AmtRead__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_amt212ev_interfaces__msg__AmtRead(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_amt212ev_interfaces
size_t max_serialized_size_amt212ev_interfaces__msg__AmtRead(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: rads
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: radps
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = amt212ev_interfaces__msg__AmtRead;
    is_plain =
      (
      offsetof(DataType, radps) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _AmtRead__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_amt212ev_interfaces__msg__AmtRead(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_AmtRead = {
  "amt212ev_interfaces::msg",
  "AmtRead",
  _AmtRead__cdr_serialize,
  _AmtRead__cdr_deserialize,
  _AmtRead__get_serialized_size,
  _AmtRead__max_serialized_size
};

static rosidl_message_type_support_t _AmtRead__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_AmtRead,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, amt212ev_interfaces, msg, AmtRead)() {
  return &_AmtRead__type_support;
}

#if defined(__cplusplus)
}
#endif

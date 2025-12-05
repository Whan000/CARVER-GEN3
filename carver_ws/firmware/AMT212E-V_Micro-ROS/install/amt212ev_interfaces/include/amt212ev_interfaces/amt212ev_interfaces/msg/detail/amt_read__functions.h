// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from amt212ev_interfaces:msg/AmtRead.idl
// generated code does not contain a copyright notice

#ifndef AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__FUNCTIONS_H_
#define AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "amt212ev_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "amt212ev_interfaces/msg/detail/amt_read__struct.h"

/// Initialize msg/AmtRead message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * amt212ev_interfaces__msg__AmtRead
 * )) before or use
 * amt212ev_interfaces__msg__AmtRead__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_amt212ev_interfaces
bool
amt212ev_interfaces__msg__AmtRead__init(amt212ev_interfaces__msg__AmtRead * msg);

/// Finalize msg/AmtRead message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_amt212ev_interfaces
void
amt212ev_interfaces__msg__AmtRead__fini(amt212ev_interfaces__msg__AmtRead * msg);

/// Create msg/AmtRead message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * amt212ev_interfaces__msg__AmtRead__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_amt212ev_interfaces
amt212ev_interfaces__msg__AmtRead *
amt212ev_interfaces__msg__AmtRead__create();

/// Destroy msg/AmtRead message.
/**
 * It calls
 * amt212ev_interfaces__msg__AmtRead__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_amt212ev_interfaces
void
amt212ev_interfaces__msg__AmtRead__destroy(amt212ev_interfaces__msg__AmtRead * msg);

/// Check for msg/AmtRead message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_amt212ev_interfaces
bool
amt212ev_interfaces__msg__AmtRead__are_equal(const amt212ev_interfaces__msg__AmtRead * lhs, const amt212ev_interfaces__msg__AmtRead * rhs);

/// Copy a msg/AmtRead message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_amt212ev_interfaces
bool
amt212ev_interfaces__msg__AmtRead__copy(
  const amt212ev_interfaces__msg__AmtRead * input,
  amt212ev_interfaces__msg__AmtRead * output);

/// Initialize array of msg/AmtRead messages.
/**
 * It allocates the memory for the number of elements and calls
 * amt212ev_interfaces__msg__AmtRead__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_amt212ev_interfaces
bool
amt212ev_interfaces__msg__AmtRead__Sequence__init(amt212ev_interfaces__msg__AmtRead__Sequence * array, size_t size);

/// Finalize array of msg/AmtRead messages.
/**
 * It calls
 * amt212ev_interfaces__msg__AmtRead__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_amt212ev_interfaces
void
amt212ev_interfaces__msg__AmtRead__Sequence__fini(amt212ev_interfaces__msg__AmtRead__Sequence * array);

/// Create array of msg/AmtRead messages.
/**
 * It allocates the memory for the array and calls
 * amt212ev_interfaces__msg__AmtRead__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_amt212ev_interfaces
amt212ev_interfaces__msg__AmtRead__Sequence *
amt212ev_interfaces__msg__AmtRead__Sequence__create(size_t size);

/// Destroy array of msg/AmtRead messages.
/**
 * It calls
 * amt212ev_interfaces__msg__AmtRead__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_amt212ev_interfaces
void
amt212ev_interfaces__msg__AmtRead__Sequence__destroy(amt212ev_interfaces__msg__AmtRead__Sequence * array);

/// Check for msg/AmtRead message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_amt212ev_interfaces
bool
amt212ev_interfaces__msg__AmtRead__Sequence__are_equal(const amt212ev_interfaces__msg__AmtRead__Sequence * lhs, const amt212ev_interfaces__msg__AmtRead__Sequence * rhs);

/// Copy an array of msg/AmtRead messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_amt212ev_interfaces
bool
amt212ev_interfaces__msg__AmtRead__Sequence__copy(
  const amt212ev_interfaces__msg__AmtRead__Sequence * input,
  amt212ev_interfaces__msg__AmtRead__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // AMT212EV_INTERFACES__MSG__DETAIL__AMT_READ__FUNCTIONS_H_

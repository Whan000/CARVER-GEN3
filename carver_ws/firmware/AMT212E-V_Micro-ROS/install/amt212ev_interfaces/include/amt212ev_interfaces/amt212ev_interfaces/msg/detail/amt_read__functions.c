// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from amt212ev_interfaces:msg/AmtRead.idl
// generated code does not contain a copyright notice
#include "amt212ev_interfaces/msg/detail/amt_read__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
amt212ev_interfaces__msg__AmtRead__init(amt212ev_interfaces__msg__AmtRead * msg)
{
  if (!msg) {
    return false;
  }
  // rads
  // radps
  return true;
}

void
amt212ev_interfaces__msg__AmtRead__fini(amt212ev_interfaces__msg__AmtRead * msg)
{
  if (!msg) {
    return;
  }
  // rads
  // radps
}

bool
amt212ev_interfaces__msg__AmtRead__are_equal(const amt212ev_interfaces__msg__AmtRead * lhs, const amt212ev_interfaces__msg__AmtRead * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // rads
  if (lhs->rads != rhs->rads) {
    return false;
  }
  // radps
  if (lhs->radps != rhs->radps) {
    return false;
  }
  return true;
}

bool
amt212ev_interfaces__msg__AmtRead__copy(
  const amt212ev_interfaces__msg__AmtRead * input,
  amt212ev_interfaces__msg__AmtRead * output)
{
  if (!input || !output) {
    return false;
  }
  // rads
  output->rads = input->rads;
  // radps
  output->radps = input->radps;
  return true;
}

amt212ev_interfaces__msg__AmtRead *
amt212ev_interfaces__msg__AmtRead__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  amt212ev_interfaces__msg__AmtRead * msg = (amt212ev_interfaces__msg__AmtRead *)allocator.allocate(sizeof(amt212ev_interfaces__msg__AmtRead), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(amt212ev_interfaces__msg__AmtRead));
  bool success = amt212ev_interfaces__msg__AmtRead__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
amt212ev_interfaces__msg__AmtRead__destroy(amt212ev_interfaces__msg__AmtRead * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    amt212ev_interfaces__msg__AmtRead__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
amt212ev_interfaces__msg__AmtRead__Sequence__init(amt212ev_interfaces__msg__AmtRead__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  amt212ev_interfaces__msg__AmtRead * data = NULL;

  if (size) {
    data = (amt212ev_interfaces__msg__AmtRead *)allocator.zero_allocate(size, sizeof(amt212ev_interfaces__msg__AmtRead), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = amt212ev_interfaces__msg__AmtRead__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        amt212ev_interfaces__msg__AmtRead__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
amt212ev_interfaces__msg__AmtRead__Sequence__fini(amt212ev_interfaces__msg__AmtRead__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      amt212ev_interfaces__msg__AmtRead__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

amt212ev_interfaces__msg__AmtRead__Sequence *
amt212ev_interfaces__msg__AmtRead__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  amt212ev_interfaces__msg__AmtRead__Sequence * array = (amt212ev_interfaces__msg__AmtRead__Sequence *)allocator.allocate(sizeof(amt212ev_interfaces__msg__AmtRead__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = amt212ev_interfaces__msg__AmtRead__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
amt212ev_interfaces__msg__AmtRead__Sequence__destroy(amt212ev_interfaces__msg__AmtRead__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    amt212ev_interfaces__msg__AmtRead__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
amt212ev_interfaces__msg__AmtRead__Sequence__are_equal(const amt212ev_interfaces__msg__AmtRead__Sequence * lhs, const amt212ev_interfaces__msg__AmtRead__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!amt212ev_interfaces__msg__AmtRead__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
amt212ev_interfaces__msg__AmtRead__Sequence__copy(
  const amt212ev_interfaces__msg__AmtRead__Sequence * input,
  amt212ev_interfaces__msg__AmtRead__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(amt212ev_interfaces__msg__AmtRead);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    amt212ev_interfaces__msg__AmtRead * data =
      (amt212ev_interfaces__msg__AmtRead *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!amt212ev_interfaces__msg__AmtRead__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          amt212ev_interfaces__msg__AmtRead__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!amt212ev_interfaces__msg__AmtRead__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

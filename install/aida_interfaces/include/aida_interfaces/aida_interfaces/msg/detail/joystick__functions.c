// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from aida_interfaces:msg/Joystick.idl
// generated code does not contain a copyright notice
#include "aida_interfaces/msg/detail/joystick__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
aida_interfaces__msg__Joystick__init(aida_interfaces__msg__Joystick * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    aida_interfaces__msg__Joystick__fini(msg);
    return false;
  }
  // x
  // y
  return true;
}

void
aida_interfaces__msg__Joystick__fini(aida_interfaces__msg__Joystick * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // x
  // y
}

bool
aida_interfaces__msg__Joystick__are_equal(const aida_interfaces__msg__Joystick * lhs, const aida_interfaces__msg__Joystick * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  return true;
}

bool
aida_interfaces__msg__Joystick__copy(
  const aida_interfaces__msg__Joystick * input,
  aida_interfaces__msg__Joystick * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  return true;
}

aida_interfaces__msg__Joystick *
aida_interfaces__msg__Joystick__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aida_interfaces__msg__Joystick * msg = (aida_interfaces__msg__Joystick *)allocator.allocate(sizeof(aida_interfaces__msg__Joystick), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aida_interfaces__msg__Joystick));
  bool success = aida_interfaces__msg__Joystick__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aida_interfaces__msg__Joystick__destroy(aida_interfaces__msg__Joystick * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aida_interfaces__msg__Joystick__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aida_interfaces__msg__Joystick__Sequence__init(aida_interfaces__msg__Joystick__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aida_interfaces__msg__Joystick * data = NULL;

  if (size) {
    data = (aida_interfaces__msg__Joystick *)allocator.zero_allocate(size, sizeof(aida_interfaces__msg__Joystick), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aida_interfaces__msg__Joystick__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aida_interfaces__msg__Joystick__fini(&data[i - 1]);
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
aida_interfaces__msg__Joystick__Sequence__fini(aida_interfaces__msg__Joystick__Sequence * array)
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
      aida_interfaces__msg__Joystick__fini(&array->data[i]);
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

aida_interfaces__msg__Joystick__Sequence *
aida_interfaces__msg__Joystick__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aida_interfaces__msg__Joystick__Sequence * array = (aida_interfaces__msg__Joystick__Sequence *)allocator.allocate(sizeof(aida_interfaces__msg__Joystick__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aida_interfaces__msg__Joystick__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aida_interfaces__msg__Joystick__Sequence__destroy(aida_interfaces__msg__Joystick__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aida_interfaces__msg__Joystick__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aida_interfaces__msg__Joystick__Sequence__are_equal(const aida_interfaces__msg__Joystick__Sequence * lhs, const aida_interfaces__msg__Joystick__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aida_interfaces__msg__Joystick__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aida_interfaces__msg__Joystick__Sequence__copy(
  const aida_interfaces__msg__Joystick__Sequence * input,
  aida_interfaces__msg__Joystick__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aida_interfaces__msg__Joystick);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    aida_interfaces__msg__Joystick * data =
      (aida_interfaces__msg__Joystick *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aida_interfaces__msg__Joystick__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          aida_interfaces__msg__Joystick__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!aida_interfaces__msg__Joystick__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

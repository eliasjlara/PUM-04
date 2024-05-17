// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lidar_data:msg/LidarData.idl
// generated code does not contain a copyright notice
#include "lidar_data/msg/detail/lidar_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
lidar_data__msg__LidarData__init(lidar_data__msg__LidarData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    lidar_data__msg__LidarData__fini(msg);
    return false;
  }
  // data
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->data, 0)) {
    lidar_data__msg__LidarData__fini(msg);
    return false;
  }
  // length
  return true;
}

void
lidar_data__msg__LidarData__fini(lidar_data__msg__LidarData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // data
  rosidl_runtime_c__int32__Sequence__fini(&msg->data);
  // length
}

bool
lidar_data__msg__LidarData__are_equal(const lidar_data__msg__LidarData * lhs, const lidar_data__msg__LidarData * rhs)
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
  // data
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  // length
  if (lhs->length != rhs->length) {
    return false;
  }
  return true;
}

bool
lidar_data__msg__LidarData__copy(
  const lidar_data__msg__LidarData * input,
  lidar_data__msg__LidarData * output)
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
  // data
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  // length
  output->length = input->length;
  return true;
}

lidar_data__msg__LidarData *
lidar_data__msg__LidarData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_data__msg__LidarData * msg = (lidar_data__msg__LidarData *)allocator.allocate(sizeof(lidar_data__msg__LidarData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lidar_data__msg__LidarData));
  bool success = lidar_data__msg__LidarData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lidar_data__msg__LidarData__destroy(lidar_data__msg__LidarData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lidar_data__msg__LidarData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lidar_data__msg__LidarData__Sequence__init(lidar_data__msg__LidarData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_data__msg__LidarData * data = NULL;

  if (size) {
    data = (lidar_data__msg__LidarData *)allocator.zero_allocate(size, sizeof(lidar_data__msg__LidarData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lidar_data__msg__LidarData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lidar_data__msg__LidarData__fini(&data[i - 1]);
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
lidar_data__msg__LidarData__Sequence__fini(lidar_data__msg__LidarData__Sequence * array)
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
      lidar_data__msg__LidarData__fini(&array->data[i]);
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

lidar_data__msg__LidarData__Sequence *
lidar_data__msg__LidarData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lidar_data__msg__LidarData__Sequence * array = (lidar_data__msg__LidarData__Sequence *)allocator.allocate(sizeof(lidar_data__msg__LidarData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lidar_data__msg__LidarData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lidar_data__msg__LidarData__Sequence__destroy(lidar_data__msg__LidarData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lidar_data__msg__LidarData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lidar_data__msg__LidarData__Sequence__are_equal(const lidar_data__msg__LidarData__Sequence * lhs, const lidar_data__msg__LidarData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lidar_data__msg__LidarData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lidar_data__msg__LidarData__Sequence__copy(
  const lidar_data__msg__LidarData__Sequence * input,
  lidar_data__msg__LidarData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lidar_data__msg__LidarData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    lidar_data__msg__LidarData * data =
      (lidar_data__msg__LidarData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lidar_data__msg__LidarData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          lidar_data__msg__LidarData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!lidar_data__msg__LidarData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

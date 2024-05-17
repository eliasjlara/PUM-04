// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from aida_interfaces:srv/SetState.idl
// generated code does not contain a copyright notice
#include "aida_interfaces/srv/detail/set_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `desired_state`
#include "rosidl_runtime_c/string_functions.h"

bool
aida_interfaces__srv__SetState_Request__init(aida_interfaces__srv__SetState_Request * msg)
{
  if (!msg) {
    return false;
  }
  // desired_state
  if (!rosidl_runtime_c__String__init(&msg->desired_state)) {
    aida_interfaces__srv__SetState_Request__fini(msg);
    return false;
  }
  return true;
}

void
aida_interfaces__srv__SetState_Request__fini(aida_interfaces__srv__SetState_Request * msg)
{
  if (!msg) {
    return;
  }
  // desired_state
  rosidl_runtime_c__String__fini(&msg->desired_state);
}

bool
aida_interfaces__srv__SetState_Request__are_equal(const aida_interfaces__srv__SetState_Request * lhs, const aida_interfaces__srv__SetState_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // desired_state
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->desired_state), &(rhs->desired_state)))
  {
    return false;
  }
  return true;
}

bool
aida_interfaces__srv__SetState_Request__copy(
  const aida_interfaces__srv__SetState_Request * input,
  aida_interfaces__srv__SetState_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // desired_state
  if (!rosidl_runtime_c__String__copy(
      &(input->desired_state), &(output->desired_state)))
  {
    return false;
  }
  return true;
}

aida_interfaces__srv__SetState_Request *
aida_interfaces__srv__SetState_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aida_interfaces__srv__SetState_Request * msg = (aida_interfaces__srv__SetState_Request *)allocator.allocate(sizeof(aida_interfaces__srv__SetState_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aida_interfaces__srv__SetState_Request));
  bool success = aida_interfaces__srv__SetState_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aida_interfaces__srv__SetState_Request__destroy(aida_interfaces__srv__SetState_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aida_interfaces__srv__SetState_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aida_interfaces__srv__SetState_Request__Sequence__init(aida_interfaces__srv__SetState_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aida_interfaces__srv__SetState_Request * data = NULL;

  if (size) {
    data = (aida_interfaces__srv__SetState_Request *)allocator.zero_allocate(size, sizeof(aida_interfaces__srv__SetState_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aida_interfaces__srv__SetState_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aida_interfaces__srv__SetState_Request__fini(&data[i - 1]);
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
aida_interfaces__srv__SetState_Request__Sequence__fini(aida_interfaces__srv__SetState_Request__Sequence * array)
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
      aida_interfaces__srv__SetState_Request__fini(&array->data[i]);
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

aida_interfaces__srv__SetState_Request__Sequence *
aida_interfaces__srv__SetState_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aida_interfaces__srv__SetState_Request__Sequence * array = (aida_interfaces__srv__SetState_Request__Sequence *)allocator.allocate(sizeof(aida_interfaces__srv__SetState_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aida_interfaces__srv__SetState_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aida_interfaces__srv__SetState_Request__Sequence__destroy(aida_interfaces__srv__SetState_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aida_interfaces__srv__SetState_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aida_interfaces__srv__SetState_Request__Sequence__are_equal(const aida_interfaces__srv__SetState_Request__Sequence * lhs, const aida_interfaces__srv__SetState_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aida_interfaces__srv__SetState_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aida_interfaces__srv__SetState_Request__Sequence__copy(
  const aida_interfaces__srv__SetState_Request__Sequence * input,
  aida_interfaces__srv__SetState_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aida_interfaces__srv__SetState_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    aida_interfaces__srv__SetState_Request * data =
      (aida_interfaces__srv__SetState_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aida_interfaces__srv__SetState_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          aida_interfaces__srv__SetState_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!aida_interfaces__srv__SetState_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
aida_interfaces__srv__SetState_Response__init(aida_interfaces__srv__SetState_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    aida_interfaces__srv__SetState_Response__fini(msg);
    return false;
  }
  return true;
}

void
aida_interfaces__srv__SetState_Response__fini(aida_interfaces__srv__SetState_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
aida_interfaces__srv__SetState_Response__are_equal(const aida_interfaces__srv__SetState_Response * lhs, const aida_interfaces__srv__SetState_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
aida_interfaces__srv__SetState_Response__copy(
  const aida_interfaces__srv__SetState_Response * input,
  aida_interfaces__srv__SetState_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

aida_interfaces__srv__SetState_Response *
aida_interfaces__srv__SetState_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aida_interfaces__srv__SetState_Response * msg = (aida_interfaces__srv__SetState_Response *)allocator.allocate(sizeof(aida_interfaces__srv__SetState_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(aida_interfaces__srv__SetState_Response));
  bool success = aida_interfaces__srv__SetState_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
aida_interfaces__srv__SetState_Response__destroy(aida_interfaces__srv__SetState_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    aida_interfaces__srv__SetState_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
aida_interfaces__srv__SetState_Response__Sequence__init(aida_interfaces__srv__SetState_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aida_interfaces__srv__SetState_Response * data = NULL;

  if (size) {
    data = (aida_interfaces__srv__SetState_Response *)allocator.zero_allocate(size, sizeof(aida_interfaces__srv__SetState_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = aida_interfaces__srv__SetState_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        aida_interfaces__srv__SetState_Response__fini(&data[i - 1]);
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
aida_interfaces__srv__SetState_Response__Sequence__fini(aida_interfaces__srv__SetState_Response__Sequence * array)
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
      aida_interfaces__srv__SetState_Response__fini(&array->data[i]);
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

aida_interfaces__srv__SetState_Response__Sequence *
aida_interfaces__srv__SetState_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  aida_interfaces__srv__SetState_Response__Sequence * array = (aida_interfaces__srv__SetState_Response__Sequence *)allocator.allocate(sizeof(aida_interfaces__srv__SetState_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = aida_interfaces__srv__SetState_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
aida_interfaces__srv__SetState_Response__Sequence__destroy(aida_interfaces__srv__SetState_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    aida_interfaces__srv__SetState_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
aida_interfaces__srv__SetState_Response__Sequence__are_equal(const aida_interfaces__srv__SetState_Response__Sequence * lhs, const aida_interfaces__srv__SetState_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!aida_interfaces__srv__SetState_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
aida_interfaces__srv__SetState_Response__Sequence__copy(
  const aida_interfaces__srv__SetState_Response__Sequence * input,
  aida_interfaces__srv__SetState_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(aida_interfaces__srv__SetState_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    aida_interfaces__srv__SetState_Response * data =
      (aida_interfaces__srv__SetState_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!aida_interfaces__srv__SetState_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          aida_interfaces__srv__SetState_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!aida_interfaces__srv__SetState_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mbot_interface:msg/Person.idl
// generated code does not contain a copyright notice
#include "mbot_interface/msg/detail/person__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

bool
mbot_interface__msg__Person__init(mbot_interface__msg__Person * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    mbot_interface__msg__Person__fini(msg);
    return false;
  }
  // age
  // height
  return true;
}

void
mbot_interface__msg__Person__fini(mbot_interface__msg__Person * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // age
  // height
}

bool
mbot_interface__msg__Person__are_equal(const mbot_interface__msg__Person * lhs, const mbot_interface__msg__Person * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // age
  if (lhs->age != rhs->age) {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  return true;
}

bool
mbot_interface__msg__Person__copy(
  const mbot_interface__msg__Person * input,
  mbot_interface__msg__Person * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // age
  output->age = input->age;
  // height
  output->height = input->height;
  return true;
}

mbot_interface__msg__Person *
mbot_interface__msg__Person__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbot_interface__msg__Person * msg = (mbot_interface__msg__Person *)allocator.allocate(sizeof(mbot_interface__msg__Person), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mbot_interface__msg__Person));
  bool success = mbot_interface__msg__Person__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mbot_interface__msg__Person__destroy(mbot_interface__msg__Person * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mbot_interface__msg__Person__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mbot_interface__msg__Person__Sequence__init(mbot_interface__msg__Person__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbot_interface__msg__Person * data = NULL;

  if (size) {
    data = (mbot_interface__msg__Person *)allocator.zero_allocate(size, sizeof(mbot_interface__msg__Person), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mbot_interface__msg__Person__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mbot_interface__msg__Person__fini(&data[i - 1]);
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
mbot_interface__msg__Person__Sequence__fini(mbot_interface__msg__Person__Sequence * array)
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
      mbot_interface__msg__Person__fini(&array->data[i]);
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

mbot_interface__msg__Person__Sequence *
mbot_interface__msg__Person__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mbot_interface__msg__Person__Sequence * array = (mbot_interface__msg__Person__Sequence *)allocator.allocate(sizeof(mbot_interface__msg__Person__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mbot_interface__msg__Person__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mbot_interface__msg__Person__Sequence__destroy(mbot_interface__msg__Person__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mbot_interface__msg__Person__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mbot_interface__msg__Person__Sequence__are_equal(const mbot_interface__msg__Person__Sequence * lhs, const mbot_interface__msg__Person__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mbot_interface__msg__Person__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mbot_interface__msg__Person__Sequence__copy(
  const mbot_interface__msg__Person__Sequence * input,
  mbot_interface__msg__Person__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mbot_interface__msg__Person);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mbot_interface__msg__Person * data =
      (mbot_interface__msg__Person *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mbot_interface__msg__Person__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mbot_interface__msg__Person__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mbot_interface__msg__Person__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}

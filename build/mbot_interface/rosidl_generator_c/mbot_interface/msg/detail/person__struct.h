// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mbot_interface:msg/Person.idl
// generated code does not contain a copyright notice

#ifndef MBOT_INTERFACE__MSG__DETAIL__PERSON__STRUCT_H_
#define MBOT_INTERFACE__MSG__DETAIL__PERSON__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Person in the package mbot_interface.
typedef struct mbot_interface__msg__Person
{
  rosidl_runtime_c__String name;
  int32_t age;
  double height;
} mbot_interface__msg__Person;

// Struct for a sequence of mbot_interface__msg__Person.
typedef struct mbot_interface__msg__Person__Sequence
{
  mbot_interface__msg__Person * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mbot_interface__msg__Person__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MBOT_INTERFACE__MSG__DETAIL__PERSON__STRUCT_H_

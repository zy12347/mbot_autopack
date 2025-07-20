// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from mbot_interface:msg/Person.idl
// generated code does not contain a copyright notice

#ifndef MBOT_INTERFACE__MSG__DETAIL__PERSON__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MBOT_INTERFACE__MSG__DETAIL__PERSON__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "mbot_interface/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "mbot_interface/msg/detail/person__struct.hpp"

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

#include "fastcdr/Cdr.h"

namespace mbot_interface
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mbot_interface
cdr_serialize(
  const mbot_interface::msg::Person & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mbot_interface
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  mbot_interface::msg::Person & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mbot_interface
get_serialized_size(
  const mbot_interface::msg::Person & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mbot_interface
max_serialized_size_Person(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace mbot_interface

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mbot_interface
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mbot_interface, msg, Person)();

#ifdef __cplusplus
}
#endif

#endif  // MBOT_INTERFACE__MSG__DETAIL__PERSON__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

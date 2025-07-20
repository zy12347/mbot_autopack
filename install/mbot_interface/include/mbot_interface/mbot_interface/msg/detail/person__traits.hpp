// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mbot_interface:msg/Person.idl
// generated code does not contain a copyright notice

#ifndef MBOT_INTERFACE__MSG__DETAIL__PERSON__TRAITS_HPP_
#define MBOT_INTERFACE__MSG__DETAIL__PERSON__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mbot_interface/msg/detail/person__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mbot_interface
{

namespace msg
{

inline void to_flow_style_yaml(
  const Person & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: age
  {
    out << "age: ";
    rosidl_generator_traits::value_to_yaml(msg.age, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Person & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: age
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "age: ";
    rosidl_generator_traits::value_to_yaml(msg.age, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Person & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mbot_interface

namespace rosidl_generator_traits
{

[[deprecated("use mbot_interface::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mbot_interface::msg::Person & msg,
  std::ostream & out, size_t indentation = 0)
{
  mbot_interface::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mbot_interface::msg::to_yaml() instead")]]
inline std::string to_yaml(const mbot_interface::msg::Person & msg)
{
  return mbot_interface::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mbot_interface::msg::Person>()
{
  return "mbot_interface::msg::Person";
}

template<>
inline const char * name<mbot_interface::msg::Person>()
{
  return "mbot_interface/msg/Person";
}

template<>
struct has_fixed_size<mbot_interface::msg::Person>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mbot_interface::msg::Person>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mbot_interface::msg::Person>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MBOT_INTERFACE__MSG__DETAIL__PERSON__TRAITS_HPP_

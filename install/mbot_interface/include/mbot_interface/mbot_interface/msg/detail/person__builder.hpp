// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mbot_interface:msg/Person.idl
// generated code does not contain a copyright notice

#ifndef MBOT_INTERFACE__MSG__DETAIL__PERSON__BUILDER_HPP_
#define MBOT_INTERFACE__MSG__DETAIL__PERSON__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mbot_interface/msg/detail/person__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mbot_interface
{

namespace msg
{

namespace builder
{

class Init_Person_height
{
public:
  explicit Init_Person_height(::mbot_interface::msg::Person & msg)
  : msg_(msg)
  {}
  ::mbot_interface::msg::Person height(::mbot_interface::msg::Person::_height_type arg)
  {
    msg_.height = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbot_interface::msg::Person msg_;
};

class Init_Person_age
{
public:
  explicit Init_Person_age(::mbot_interface::msg::Person & msg)
  : msg_(msg)
  {}
  Init_Person_height age(::mbot_interface::msg::Person::_age_type arg)
  {
    msg_.age = std::move(arg);
    return Init_Person_height(msg_);
  }

private:
  ::mbot_interface::msg::Person msg_;
};

class Init_Person_name
{
public:
  Init_Person_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Person_age name(::mbot_interface::msg::Person::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_Person_age(msg_);
  }

private:
  ::mbot_interface::msg::Person msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbot_interface::msg::Person>()
{
  return mbot_interface::msg::builder::Init_Person_name();
}

}  // namespace mbot_interface

#endif  // MBOT_INTERFACE__MSG__DETAIL__PERSON__BUILDER_HPP_

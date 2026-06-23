// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from asv_interfaces:msg/StateObserver.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__STATE_OBSERVER__TRAITS_HPP_
#define ASV_INTERFACES__MSG__DETAIL__STATE_OBSERVER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "asv_interfaces/msg/detail/state_observer__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'point'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'velocity'
// Member 'disturbances'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace asv_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const StateObserver & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: point
  {
    out << "point: ";
    to_flow_style_yaml(msg.point, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    to_flow_style_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: disturbances
  {
    out << "disturbances: ";
    to_flow_style_yaml(msg.disturbances, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StateObserver & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: point
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "point:\n";
    to_block_style_yaml(msg.point, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_block_style_yaml(msg.velocity, out, indentation + 2);
  }

  // member: disturbances
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "disturbances:\n";
    to_block_style_yaml(msg.disturbances, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StateObserver & msg, bool use_flow_style = false)
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

}  // namespace asv_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use asv_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const asv_interfaces::msg::StateObserver & msg,
  std::ostream & out, size_t indentation = 0)
{
  asv_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use asv_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const asv_interfaces::msg::StateObserver & msg)
{
  return asv_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<asv_interfaces::msg::StateObserver>()
{
  return "asv_interfaces::msg::StateObserver";
}

template<>
inline const char * name<asv_interfaces::msg::StateObserver>()
{
  return "asv_interfaces/msg/StateObserver";
}

template<>
struct has_fixed_size<asv_interfaces::msg::StateObserver>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<asv_interfaces::msg::StateObserver>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<asv_interfaces::msg::StateObserver>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASV_INTERFACES__MSG__DETAIL__STATE_OBSERVER__TRAITS_HPP_

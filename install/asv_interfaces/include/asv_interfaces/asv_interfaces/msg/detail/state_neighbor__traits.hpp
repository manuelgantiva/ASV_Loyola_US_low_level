// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from asv_interfaces:msg/StateNeighbor.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__STATE_NEIGHBOR__TRAITS_HPP_
#define ASV_INTERFACES__MSG__DETAIL__STATE_NEIGHBOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "asv_interfaces/msg/detail/state_neighbor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'point'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace asv_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const StateNeighbor & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
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

  // member: msg_from
  {
    out << "msg_from: ";
    rosidl_generator_traits::value_to_yaml(msg.msg_from, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const StateNeighbor & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
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

  // member: msg_from
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "msg_from: ";
    rosidl_generator_traits::value_to_yaml(msg.msg_from, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StateNeighbor & msg, bool use_flow_style = false)
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
  const asv_interfaces::msg::StateNeighbor & msg,
  std::ostream & out, size_t indentation = 0)
{
  asv_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use asv_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const asv_interfaces::msg::StateNeighbor & msg)
{
  return asv_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<asv_interfaces::msg::StateNeighbor>()
{
  return "asv_interfaces::msg::StateNeighbor";
}

template<>
inline const char * name<asv_interfaces::msg::StateNeighbor>()
{
  return "asv_interfaces/msg/StateNeighbor";
}

template<>
struct has_fixed_size<asv_interfaces::msg::StateNeighbor>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<asv_interfaces::msg::StateNeighbor>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<asv_interfaces::msg::StateNeighbor>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASV_INTERFACES__MSG__DETAIL__STATE_NEIGHBOR__TRAITS_HPP_

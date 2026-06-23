// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from asv_interfaces:msg/PwmValues.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__PWM_VALUES__TRAITS_HPP_
#define ASV_INTERFACES__MSG__DETAIL__PWM_VALUES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "asv_interfaces/msg/detail/pwm_values__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace asv_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const PwmValues & msg,
  std::ostream & out)
{
  out << "{";
  // member: t_left
  {
    out << "t_left: ";
    rosidl_generator_traits::value_to_yaml(msg.t_left, out);
    out << ", ";
  }

  // member: t_righ
  {
    out << "t_righ: ";
    rosidl_generator_traits::value_to_yaml(msg.t_righ, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PwmValues & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: t_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t_left: ";
    rosidl_generator_traits::value_to_yaml(msg.t_left, out);
    out << "\n";
  }

  // member: t_righ
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "t_righ: ";
    rosidl_generator_traits::value_to_yaml(msg.t_righ, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PwmValues & msg, bool use_flow_style = false)
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
  const asv_interfaces::msg::PwmValues & msg,
  std::ostream & out, size_t indentation = 0)
{
  asv_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use asv_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const asv_interfaces::msg::PwmValues & msg)
{
  return asv_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<asv_interfaces::msg::PwmValues>()
{
  return "asv_interfaces::msg::PwmValues";
}

template<>
inline const char * name<asv_interfaces::msg::PwmValues>()
{
  return "asv_interfaces/msg/PwmValues";
}

template<>
struct has_fixed_size<asv_interfaces::msg::PwmValues>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<asv_interfaces::msg::PwmValues>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<asv_interfaces::msg::PwmValues>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASV_INTERFACES__MSG__DETAIL__PWM_VALUES__TRAITS_HPP_

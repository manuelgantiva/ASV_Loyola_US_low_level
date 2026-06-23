// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from asv_interfaces:msg/ReferenceLlc.idl
// generated code does not contain a copyright notice

#ifndef ASV_INTERFACES__MSG__DETAIL__REFERENCE_LLC__TRAITS_HPP_
#define ASV_INTERFACES__MSG__DETAIL__REFERENCE_LLC__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "asv_interfaces/msg/detail/reference_llc__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'references'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'u_tar'
#include "std_msgs/msg/detail/float64__traits.hpp"

namespace asv_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ReferenceLlc & msg,
  std::ostream & out)
{
  out << "{";
  // member: references
  {
    if (msg.references.size() == 0) {
      out << "references: []";
    } else {
      out << "references: [";
      size_t pending_items = msg.references.size();
      for (auto item : msg.references) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: u_tar
  {
    out << "u_tar: ";
    to_flow_style_yaml(msg.u_tar, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ReferenceLlc & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: references
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.references.size() == 0) {
      out << "references: []\n";
    } else {
      out << "references:\n";
      for (auto item : msg.references) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: u_tar
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "u_tar:\n";
    to_block_style_yaml(msg.u_tar, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ReferenceLlc & msg, bool use_flow_style = false)
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
  const asv_interfaces::msg::ReferenceLlc & msg,
  std::ostream & out, size_t indentation = 0)
{
  asv_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use asv_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const asv_interfaces::msg::ReferenceLlc & msg)
{
  return asv_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<asv_interfaces::msg::ReferenceLlc>()
{
  return "asv_interfaces::msg::ReferenceLlc";
}

template<>
inline const char * name<asv_interfaces::msg::ReferenceLlc>()
{
  return "asv_interfaces/msg/ReferenceLlc";
}

template<>
struct has_fixed_size<asv_interfaces::msg::ReferenceLlc>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<asv_interfaces::msg::ReferenceLlc>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<asv_interfaces::msg::ReferenceLlc>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ASV_INTERFACES__MSG__DETAIL__REFERENCE_LLC__TRAITS_HPP_

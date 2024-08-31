// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from visibility_graph_msg:msg/Graph.idl
// generated code does not contain a copyright notice

#ifndef VISIBILITY_GRAPH_MSG__MSG__DETAIL__GRAPH__TRAITS_HPP_
#define VISIBILITY_GRAPH_MSG__MSG__DETAIL__GRAPH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "visibility_graph_msg/msg/detail/graph__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'nodes'
#include "visibility_graph_msg/msg/detail/node__traits.hpp"

namespace visibility_graph_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Graph & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: nodes
  {
    if (msg.nodes.size() == 0) {
      out << "nodes: []";
    } else {
      out << "nodes: [";
      size_t pending_items = msg.nodes.size();
      for (auto item : msg.nodes) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: size
  {
    out << "size: ";
    rosidl_generator_traits::value_to_yaml(msg.size, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Graph & msg,
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

  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: nodes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.nodes.size() == 0) {
      out << "nodes: []\n";
    } else {
      out << "nodes:\n";
      for (auto item : msg.nodes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "size: ";
    rosidl_generator_traits::value_to_yaml(msg.size, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Graph & msg, bool use_flow_style = false)
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

}  // namespace visibility_graph_msg

namespace rosidl_generator_traits
{

[[deprecated("use visibility_graph_msg::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const visibility_graph_msg::msg::Graph & msg,
  std::ostream & out, size_t indentation = 0)
{
  visibility_graph_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use visibility_graph_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const visibility_graph_msg::msg::Graph & msg)
{
  return visibility_graph_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<visibility_graph_msg::msg::Graph>()
{
  return "visibility_graph_msg::msg::Graph";
}

template<>
inline const char * name<visibility_graph_msg::msg::Graph>()
{
  return "visibility_graph_msg/msg/Graph";
}

template<>
struct has_fixed_size<visibility_graph_msg::msg::Graph>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<visibility_graph_msg::msg::Graph>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<visibility_graph_msg::msg::Graph>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VISIBILITY_GRAPH_MSG__MSG__DETAIL__GRAPH__TRAITS_HPP_

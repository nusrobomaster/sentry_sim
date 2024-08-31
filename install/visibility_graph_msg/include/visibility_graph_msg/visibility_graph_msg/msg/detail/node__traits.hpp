// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from visibility_graph_msg:msg/Node.idl
// generated code does not contain a copyright notice

#ifndef VISIBILITY_GRAPH_MSG__MSG__DETAIL__NODE__TRAITS_HPP_
#define VISIBILITY_GRAPH_MSG__MSG__DETAIL__NODE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "visibility_graph_msg/msg/detail/node__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'position'
// Member 'surface_dirs'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace visibility_graph_msg
{

namespace msg
{

inline void to_flow_style_yaml(
  const Node & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: freetype
  {
    out << "freetype: ";
    rosidl_generator_traits::value_to_yaml(msg.freetype, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: surface_dirs
  {
    if (msg.surface_dirs.size() == 0) {
      out << "surface_dirs: []";
    } else {
      out << "surface_dirs: [";
      size_t pending_items = msg.surface_dirs.size();
      for (auto item : msg.surface_dirs) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: is_covered
  {
    out << "is_covered: ";
    rosidl_generator_traits::value_to_yaml(msg.is_covered, out);
    out << ", ";
  }

  // member: is_frontier
  {
    out << "is_frontier: ";
    rosidl_generator_traits::value_to_yaml(msg.is_frontier, out);
    out << ", ";
  }

  // member: is_navpoint
  {
    out << "is_navpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.is_navpoint, out);
    out << ", ";
  }

  // member: is_boundary
  {
    out << "is_boundary: ";
    rosidl_generator_traits::value_to_yaml(msg.is_boundary, out);
    out << ", ";
  }

  // member: connect_nodes
  {
    if (msg.connect_nodes.size() == 0) {
      out << "connect_nodes: []";
    } else {
      out << "connect_nodes: [";
      size_t pending_items = msg.connect_nodes.size();
      for (auto item : msg.connect_nodes) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: poly_connects
  {
    if (msg.poly_connects.size() == 0) {
      out << "poly_connects: []";
    } else {
      out << "poly_connects: [";
      size_t pending_items = msg.poly_connects.size();
      for (auto item : msg.poly_connects) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: contour_connects
  {
    if (msg.contour_connects.size() == 0) {
      out << "contour_connects: []";
    } else {
      out << "contour_connects: [";
      size_t pending_items = msg.contour_connects.size();
      for (auto item : msg.contour_connects) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: trajectory_connects
  {
    if (msg.trajectory_connects.size() == 0) {
      out << "trajectory_connects: []";
    } else {
      out << "trajectory_connects: [";
      size_t pending_items = msg.trajectory_connects.size();
      for (auto item : msg.trajectory_connects) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Node & msg,
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

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: freetype
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "freetype: ";
    rosidl_generator_traits::value_to_yaml(msg.freetype, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: surface_dirs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.surface_dirs.size() == 0) {
      out << "surface_dirs: []\n";
    } else {
      out << "surface_dirs:\n";
      for (auto item : msg.surface_dirs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: is_covered
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_covered: ";
    rosidl_generator_traits::value_to_yaml(msg.is_covered, out);
    out << "\n";
  }

  // member: is_frontier
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_frontier: ";
    rosidl_generator_traits::value_to_yaml(msg.is_frontier, out);
    out << "\n";
  }

  // member: is_navpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_navpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.is_navpoint, out);
    out << "\n";
  }

  // member: is_boundary
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_boundary: ";
    rosidl_generator_traits::value_to_yaml(msg.is_boundary, out);
    out << "\n";
  }

  // member: connect_nodes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.connect_nodes.size() == 0) {
      out << "connect_nodes: []\n";
    } else {
      out << "connect_nodes:\n";
      for (auto item : msg.connect_nodes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: poly_connects
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poly_connects.size() == 0) {
      out << "poly_connects: []\n";
    } else {
      out << "poly_connects:\n";
      for (auto item : msg.poly_connects) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: contour_connects
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.contour_connects.size() == 0) {
      out << "contour_connects: []\n";
    } else {
      out << "contour_connects:\n";
      for (auto item : msg.contour_connects) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: trajectory_connects
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.trajectory_connects.size() == 0) {
      out << "trajectory_connects: []\n";
    } else {
      out << "trajectory_connects:\n";
      for (auto item : msg.trajectory_connects) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Node & msg, bool use_flow_style = false)
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
  const visibility_graph_msg::msg::Node & msg,
  std::ostream & out, size_t indentation = 0)
{
  visibility_graph_msg::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use visibility_graph_msg::msg::to_yaml() instead")]]
inline std::string to_yaml(const visibility_graph_msg::msg::Node & msg)
{
  return visibility_graph_msg::msg::to_yaml(msg);
}

template<>
inline const char * data_type<visibility_graph_msg::msg::Node>()
{
  return "visibility_graph_msg::msg::Node";
}

template<>
inline const char * name<visibility_graph_msg::msg::Node>()
{
  return "visibility_graph_msg/msg/Node";
}

template<>
struct has_fixed_size<visibility_graph_msg::msg::Node>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<visibility_graph_msg::msg::Node>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<visibility_graph_msg::msg::Node>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VISIBILITY_GRAPH_MSG__MSG__DETAIL__NODE__TRAITS_HPP_

// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from visibility_graph_msg:msg/Node.idl
// generated code does not contain a copyright notice

#ifndef VISIBILITY_GRAPH_MSG__MSG__DETAIL__NODE__BUILDER_HPP_
#define VISIBILITY_GRAPH_MSG__MSG__DETAIL__NODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "visibility_graph_msg/msg/detail/node__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace visibility_graph_msg
{

namespace msg
{

namespace builder
{

class Init_Node_trajectory_connects
{
public:
  explicit Init_Node_trajectory_connects(::visibility_graph_msg::msg::Node & msg)
  : msg_(msg)
  {}
  ::visibility_graph_msg::msg::Node trajectory_connects(::visibility_graph_msg::msg::Node::_trajectory_connects_type arg)
  {
    msg_.trajectory_connects = std::move(arg);
    return std::move(msg_);
  }

private:
  ::visibility_graph_msg::msg::Node msg_;
};

class Init_Node_contour_connects
{
public:
  explicit Init_Node_contour_connects(::visibility_graph_msg::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_trajectory_connects contour_connects(::visibility_graph_msg::msg::Node::_contour_connects_type arg)
  {
    msg_.contour_connects = std::move(arg);
    return Init_Node_trajectory_connects(msg_);
  }

private:
  ::visibility_graph_msg::msg::Node msg_;
};

class Init_Node_poly_connects
{
public:
  explicit Init_Node_poly_connects(::visibility_graph_msg::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_contour_connects poly_connects(::visibility_graph_msg::msg::Node::_poly_connects_type arg)
  {
    msg_.poly_connects = std::move(arg);
    return Init_Node_contour_connects(msg_);
  }

private:
  ::visibility_graph_msg::msg::Node msg_;
};

class Init_Node_connect_nodes
{
public:
  explicit Init_Node_connect_nodes(::visibility_graph_msg::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_poly_connects connect_nodes(::visibility_graph_msg::msg::Node::_connect_nodes_type arg)
  {
    msg_.connect_nodes = std::move(arg);
    return Init_Node_poly_connects(msg_);
  }

private:
  ::visibility_graph_msg::msg::Node msg_;
};

class Init_Node_is_boundary
{
public:
  explicit Init_Node_is_boundary(::visibility_graph_msg::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_connect_nodes is_boundary(::visibility_graph_msg::msg::Node::_is_boundary_type arg)
  {
    msg_.is_boundary = std::move(arg);
    return Init_Node_connect_nodes(msg_);
  }

private:
  ::visibility_graph_msg::msg::Node msg_;
};

class Init_Node_is_navpoint
{
public:
  explicit Init_Node_is_navpoint(::visibility_graph_msg::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_is_boundary is_navpoint(::visibility_graph_msg::msg::Node::_is_navpoint_type arg)
  {
    msg_.is_navpoint = std::move(arg);
    return Init_Node_is_boundary(msg_);
  }

private:
  ::visibility_graph_msg::msg::Node msg_;
};

class Init_Node_is_frontier
{
public:
  explicit Init_Node_is_frontier(::visibility_graph_msg::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_is_navpoint is_frontier(::visibility_graph_msg::msg::Node::_is_frontier_type arg)
  {
    msg_.is_frontier = std::move(arg);
    return Init_Node_is_navpoint(msg_);
  }

private:
  ::visibility_graph_msg::msg::Node msg_;
};

class Init_Node_is_covered
{
public:
  explicit Init_Node_is_covered(::visibility_graph_msg::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_is_frontier is_covered(::visibility_graph_msg::msg::Node::_is_covered_type arg)
  {
    msg_.is_covered = std::move(arg);
    return Init_Node_is_frontier(msg_);
  }

private:
  ::visibility_graph_msg::msg::Node msg_;
};

class Init_Node_surface_dirs
{
public:
  explicit Init_Node_surface_dirs(::visibility_graph_msg::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_is_covered surface_dirs(::visibility_graph_msg::msg::Node::_surface_dirs_type arg)
  {
    msg_.surface_dirs = std::move(arg);
    return Init_Node_is_covered(msg_);
  }

private:
  ::visibility_graph_msg::msg::Node msg_;
};

class Init_Node_position
{
public:
  explicit Init_Node_position(::visibility_graph_msg::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_surface_dirs position(::visibility_graph_msg::msg::Node::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_Node_surface_dirs(msg_);
  }

private:
  ::visibility_graph_msg::msg::Node msg_;
};

class Init_Node_freetype
{
public:
  explicit Init_Node_freetype(::visibility_graph_msg::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_position freetype(::visibility_graph_msg::msg::Node::_freetype_type arg)
  {
    msg_.freetype = std::move(arg);
    return Init_Node_position(msg_);
  }

private:
  ::visibility_graph_msg::msg::Node msg_;
};

class Init_Node_id
{
public:
  explicit Init_Node_id(::visibility_graph_msg::msg::Node & msg)
  : msg_(msg)
  {}
  Init_Node_freetype id(::visibility_graph_msg::msg::Node::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Node_freetype(msg_);
  }

private:
  ::visibility_graph_msg::msg::Node msg_;
};

class Init_Node_header
{
public:
  Init_Node_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Node_id header(::visibility_graph_msg::msg::Node::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Node_id(msg_);
  }

private:
  ::visibility_graph_msg::msg::Node msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::visibility_graph_msg::msg::Node>()
{
  return visibility_graph_msg::msg::builder::Init_Node_header();
}

}  // namespace visibility_graph_msg

#endif  // VISIBILITY_GRAPH_MSG__MSG__DETAIL__NODE__BUILDER_HPP_

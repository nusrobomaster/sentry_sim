// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from visibility_graph_msg:msg/Graph.idl
// generated code does not contain a copyright notice

#ifndef VISIBILITY_GRAPH_MSG__MSG__DETAIL__GRAPH__BUILDER_HPP_
#define VISIBILITY_GRAPH_MSG__MSG__DETAIL__GRAPH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "visibility_graph_msg/msg/detail/graph__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace visibility_graph_msg
{

namespace msg
{

namespace builder
{

class Init_Graph_size
{
public:
  explicit Init_Graph_size(::visibility_graph_msg::msg::Graph & msg)
  : msg_(msg)
  {}
  ::visibility_graph_msg::msg::Graph size(::visibility_graph_msg::msg::Graph::_size_type arg)
  {
    msg_.size = std::move(arg);
    return std::move(msg_);
  }

private:
  ::visibility_graph_msg::msg::Graph msg_;
};

class Init_Graph_nodes
{
public:
  explicit Init_Graph_nodes(::visibility_graph_msg::msg::Graph & msg)
  : msg_(msg)
  {}
  Init_Graph_size nodes(::visibility_graph_msg::msg::Graph::_nodes_type arg)
  {
    msg_.nodes = std::move(arg);
    return Init_Graph_size(msg_);
  }

private:
  ::visibility_graph_msg::msg::Graph msg_;
};

class Init_Graph_robot_id
{
public:
  explicit Init_Graph_robot_id(::visibility_graph_msg::msg::Graph & msg)
  : msg_(msg)
  {}
  Init_Graph_nodes robot_id(::visibility_graph_msg::msg::Graph::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_Graph_nodes(msg_);
  }

private:
  ::visibility_graph_msg::msg::Graph msg_;
};

class Init_Graph_header
{
public:
  Init_Graph_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Graph_robot_id header(::visibility_graph_msg::msg::Graph::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Graph_robot_id(msg_);
  }

private:
  ::visibility_graph_msg::msg::Graph msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::visibility_graph_msg::msg::Graph>()
{
  return visibility_graph_msg::msg::builder::Init_Graph_header();
}

}  // namespace visibility_graph_msg

#endif  // VISIBILITY_GRAPH_MSG__MSG__DETAIL__GRAPH__BUILDER_HPP_

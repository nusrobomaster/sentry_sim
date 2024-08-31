// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from visibility_graph_msg:msg/Graph.idl
// generated code does not contain a copyright notice

#ifndef VISIBILITY_GRAPH_MSG__MSG__DETAIL__GRAPH__STRUCT_HPP_
#define VISIBILITY_GRAPH_MSG__MSG__DETAIL__GRAPH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'nodes'
#include "visibility_graph_msg/msg/detail/node__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__visibility_graph_msg__msg__Graph __attribute__((deprecated))
#else
# define DEPRECATED__visibility_graph_msg__msg__Graph __declspec(deprecated)
#endif

namespace visibility_graph_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Graph_
{
  using Type = Graph_<ContainerAllocator>;

  explicit Graph_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0;
      this->size = 0ul;
    }
  }

  explicit Graph_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0;
      this->size = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _robot_id_type =
    uint16_t;
  _robot_id_type robot_id;
  using _nodes_type =
    std::vector<visibility_graph_msg::msg::Node_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visibility_graph_msg::msg::Node_<ContainerAllocator>>>;
  _nodes_type nodes;
  using _size_type =
    uint32_t;
  _size_type size;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__robot_id(
    const uint16_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__nodes(
    const std::vector<visibility_graph_msg::msg::Node_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<visibility_graph_msg::msg::Node_<ContainerAllocator>>> & _arg)
  {
    this->nodes = _arg;
    return *this;
  }
  Type & set__size(
    const uint32_t & _arg)
  {
    this->size = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    visibility_graph_msg::msg::Graph_<ContainerAllocator> *;
  using ConstRawPtr =
    const visibility_graph_msg::msg::Graph_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<visibility_graph_msg::msg::Graph_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<visibility_graph_msg::msg::Graph_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      visibility_graph_msg::msg::Graph_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<visibility_graph_msg::msg::Graph_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      visibility_graph_msg::msg::Graph_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<visibility_graph_msg::msg::Graph_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<visibility_graph_msg::msg::Graph_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<visibility_graph_msg::msg::Graph_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__visibility_graph_msg__msg__Graph
    std::shared_ptr<visibility_graph_msg::msg::Graph_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__visibility_graph_msg__msg__Graph
    std::shared_ptr<visibility_graph_msg::msg::Graph_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Graph_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->nodes != other.nodes) {
      return false;
    }
    if (this->size != other.size) {
      return false;
    }
    return true;
  }
  bool operator!=(const Graph_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Graph_

// alias to use template instance with default allocator
using Graph =
  visibility_graph_msg::msg::Graph_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace visibility_graph_msg

#endif  // VISIBILITY_GRAPH_MSG__MSG__DETAIL__GRAPH__STRUCT_HPP_

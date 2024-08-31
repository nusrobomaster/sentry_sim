// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from visibility_graph_msg:msg/Node.idl
// generated code does not contain a copyright notice

#ifndef VISIBILITY_GRAPH_MSG__MSG__DETAIL__NODE__STRUCT_HPP_
#define VISIBILITY_GRAPH_MSG__MSG__DETAIL__NODE__STRUCT_HPP_

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
// Member 'position'
// Member 'surface_dirs'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__visibility_graph_msg__msg__Node __attribute__((deprecated))
#else
# define DEPRECATED__visibility_graph_msg__msg__Node __declspec(deprecated)
#endif

namespace visibility_graph_msg
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Node_
{
  using Type = Node_<ContainerAllocator>;

  explicit Node_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->freetype = 0;
      this->is_covered = false;
      this->is_frontier = false;
      this->is_navpoint = false;
      this->is_boundary = false;
    }
  }

  explicit Node_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ul;
      this->freetype = 0;
      this->is_covered = false;
      this->is_frontier = false;
      this->is_navpoint = false;
      this->is_boundary = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_type =
    uint32_t;
  _id_type id;
  using _freetype_type =
    uint8_t;
  _freetype_type freetype;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _surface_dirs_type =
    std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>>;
  _surface_dirs_type surface_dirs;
  using _is_covered_type =
    bool;
  _is_covered_type is_covered;
  using _is_frontier_type =
    bool;
  _is_frontier_type is_frontier;
  using _is_navpoint_type =
    bool;
  _is_navpoint_type is_navpoint;
  using _is_boundary_type =
    bool;
  _is_boundary_type is_boundary;
  using _connect_nodes_type =
    std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>>;
  _connect_nodes_type connect_nodes;
  using _poly_connects_type =
    std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>>;
  _poly_connects_type poly_connects;
  using _contour_connects_type =
    std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>>;
  _contour_connects_type contour_connects;
  using _trajectory_connects_type =
    std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>>;
  _trajectory_connects_type trajectory_connects;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__id(
    const uint32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__freetype(
    const uint8_t & _arg)
  {
    this->freetype = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__surface_dirs(
    const std::vector<geometry_msgs::msg::Point_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Point_<ContainerAllocator>>> & _arg)
  {
    this->surface_dirs = _arg;
    return *this;
  }
  Type & set__is_covered(
    const bool & _arg)
  {
    this->is_covered = _arg;
    return *this;
  }
  Type & set__is_frontier(
    const bool & _arg)
  {
    this->is_frontier = _arg;
    return *this;
  }
  Type & set__is_navpoint(
    const bool & _arg)
  {
    this->is_navpoint = _arg;
    return *this;
  }
  Type & set__is_boundary(
    const bool & _arg)
  {
    this->is_boundary = _arg;
    return *this;
  }
  Type & set__connect_nodes(
    const std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> & _arg)
  {
    this->connect_nodes = _arg;
    return *this;
  }
  Type & set__poly_connects(
    const std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> & _arg)
  {
    this->poly_connects = _arg;
    return *this;
  }
  Type & set__contour_connects(
    const std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> & _arg)
  {
    this->contour_connects = _arg;
    return *this;
  }
  Type & set__trajectory_connects(
    const std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> & _arg)
  {
    this->trajectory_connects = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    visibility_graph_msg::msg::Node_<ContainerAllocator> *;
  using ConstRawPtr =
    const visibility_graph_msg::msg::Node_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<visibility_graph_msg::msg::Node_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<visibility_graph_msg::msg::Node_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      visibility_graph_msg::msg::Node_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<visibility_graph_msg::msg::Node_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      visibility_graph_msg::msg::Node_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<visibility_graph_msg::msg::Node_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<visibility_graph_msg::msg::Node_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<visibility_graph_msg::msg::Node_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__visibility_graph_msg__msg__Node
    std::shared_ptr<visibility_graph_msg::msg::Node_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__visibility_graph_msg__msg__Node
    std::shared_ptr<visibility_graph_msg::msg::Node_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Node_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->freetype != other.freetype) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->surface_dirs != other.surface_dirs) {
      return false;
    }
    if (this->is_covered != other.is_covered) {
      return false;
    }
    if (this->is_frontier != other.is_frontier) {
      return false;
    }
    if (this->is_navpoint != other.is_navpoint) {
      return false;
    }
    if (this->is_boundary != other.is_boundary) {
      return false;
    }
    if (this->connect_nodes != other.connect_nodes) {
      return false;
    }
    if (this->poly_connects != other.poly_connects) {
      return false;
    }
    if (this->contour_connects != other.contour_connects) {
      return false;
    }
    if (this->trajectory_connects != other.trajectory_connects) {
      return false;
    }
    return true;
  }
  bool operator!=(const Node_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Node_

// alias to use template instance with default allocator
using Node =
  visibility_graph_msg::msg::Node_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace visibility_graph_msg

#endif  // VISIBILITY_GRAPH_MSG__MSG__DETAIL__NODE__STRUCT_HPP_

// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from visibility_graph_msg:msg/Node.idl
// generated code does not contain a copyright notice

#ifndef VISIBILITY_GRAPH_MSG__MSG__DETAIL__NODE__STRUCT_H_
#define VISIBILITY_GRAPH_MSG__MSG__DETAIL__NODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'position'
// Member 'surface_dirs'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'connect_nodes'
// Member 'poly_connects'
// Member 'contour_connects'
// Member 'trajectory_connects'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Node in the package visibility_graph_msg.
typedef struct visibility_graph_msg__msg__Node
{
  std_msgs__msg__Header header;
  uint32_t id;
  uint8_t freetype;
  geometry_msgs__msg__Point position;
  geometry_msgs__msg__Point__Sequence surface_dirs;
  bool is_covered;
  bool is_frontier;
  bool is_navpoint;
  bool is_boundary;
  rosidl_runtime_c__uint32__Sequence connect_nodes;
  rosidl_runtime_c__uint32__Sequence poly_connects;
  rosidl_runtime_c__uint32__Sequence contour_connects;
  rosidl_runtime_c__uint32__Sequence trajectory_connects;
} visibility_graph_msg__msg__Node;

// Struct for a sequence of visibility_graph_msg__msg__Node.
typedef struct visibility_graph_msg__msg__Node__Sequence
{
  visibility_graph_msg__msg__Node * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} visibility_graph_msg__msg__Node__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VISIBILITY_GRAPH_MSG__MSG__DETAIL__NODE__STRUCT_H_

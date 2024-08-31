// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from visibility_graph_msg:msg/Graph.idl
// generated code does not contain a copyright notice

#ifndef VISIBILITY_GRAPH_MSG__MSG__DETAIL__GRAPH__STRUCT_H_
#define VISIBILITY_GRAPH_MSG__MSG__DETAIL__GRAPH__STRUCT_H_

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
// Member 'nodes'
#include "visibility_graph_msg/msg/detail/node__struct.h"

/// Struct defined in msg/Graph in the package visibility_graph_msg.
typedef struct visibility_graph_msg__msg__Graph
{
  std_msgs__msg__Header header;
  uint16_t robot_id;
  visibility_graph_msg__msg__Node__Sequence nodes;
  uint32_t size;
} visibility_graph_msg__msg__Graph;

// Struct for a sequence of visibility_graph_msg__msg__Graph.
typedef struct visibility_graph_msg__msg__Graph__Sequence
{
  visibility_graph_msg__msg__Graph * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} visibility_graph_msg__msg__Graph__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VISIBILITY_GRAPH_MSG__MSG__DETAIL__GRAPH__STRUCT_H_

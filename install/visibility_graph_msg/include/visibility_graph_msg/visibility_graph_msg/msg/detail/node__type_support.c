// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from visibility_graph_msg:msg/Node.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "visibility_graph_msg/msg/detail/node__rosidl_typesupport_introspection_c.h"
#include "visibility_graph_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "visibility_graph_msg/msg/detail/node__functions.h"
#include "visibility_graph_msg/msg/detail/node__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `position`
// Member `surface_dirs`
#include "geometry_msgs/msg/point.h"
// Member `position`
// Member `surface_dirs`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `connect_nodes`
// Member `poly_connects`
// Member `contour_connects`
// Member `trajectory_connects`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__Node_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  visibility_graph_msg__msg__Node__init(message_memory);
}

void visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__Node_fini_function(void * message_memory)
{
  visibility_graph_msg__msg__Node__fini(message_memory);
}

size_t visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__size_function__Node__surface_dirs(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_const_function__Node__surface_dirs(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_function__Node__surface_dirs(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__fetch_function__Node__surface_dirs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_const_function__Node__surface_dirs(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__assign_function__Node__surface_dirs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_function__Node__surface_dirs(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__resize_function__Node__surface_dirs(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

size_t visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__size_function__Node__connect_nodes(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return member->size;
}

const void * visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_const_function__Node__connect_nodes(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_function__Node__connect_nodes(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__fetch_function__Node__connect_nodes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_const_function__Node__connect_nodes(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__assign_function__Node__connect_nodes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_function__Node__connect_nodes(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

bool visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__resize_function__Node__connect_nodes(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  rosidl_runtime_c__uint32__Sequence__fini(member);
  return rosidl_runtime_c__uint32__Sequence__init(member, size);
}

size_t visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__size_function__Node__poly_connects(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return member->size;
}

const void * visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_const_function__Node__poly_connects(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_function__Node__poly_connects(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__fetch_function__Node__poly_connects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_const_function__Node__poly_connects(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__assign_function__Node__poly_connects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_function__Node__poly_connects(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

bool visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__resize_function__Node__poly_connects(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  rosidl_runtime_c__uint32__Sequence__fini(member);
  return rosidl_runtime_c__uint32__Sequence__init(member, size);
}

size_t visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__size_function__Node__contour_connects(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return member->size;
}

const void * visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_const_function__Node__contour_connects(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_function__Node__contour_connects(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__fetch_function__Node__contour_connects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_const_function__Node__contour_connects(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__assign_function__Node__contour_connects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_function__Node__contour_connects(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

bool visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__resize_function__Node__contour_connects(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  rosidl_runtime_c__uint32__Sequence__fini(member);
  return rosidl_runtime_c__uint32__Sequence__init(member, size);
}

size_t visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__size_function__Node__trajectory_connects(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return member->size;
}

const void * visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_const_function__Node__trajectory_connects(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_function__Node__trajectory_connects(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__fetch_function__Node__trajectory_connects(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_const_function__Node__trajectory_connects(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__assign_function__Node__trajectory_connects(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_function__Node__trajectory_connects(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

bool visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__resize_function__Node__trajectory_connects(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  rosidl_runtime_c__uint32__Sequence__fini(member);
  return rosidl_runtime_c__uint32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__Node_message_member_array[13] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Node, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Node, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "freetype",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Node, freetype),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Node, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "surface_dirs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Node, surface_dirs),  // bytes offset in struct
    NULL,  // default value
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__size_function__Node__surface_dirs,  // size() function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_const_function__Node__surface_dirs,  // get_const(index) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_function__Node__surface_dirs,  // get(index) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__fetch_function__Node__surface_dirs,  // fetch(index, &value) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__assign_function__Node__surface_dirs,  // assign(index, value) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__resize_function__Node__surface_dirs  // resize(index) function pointer
  },
  {
    "is_covered",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Node, is_covered),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_frontier",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Node, is_frontier),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_navpoint",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Node, is_navpoint),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_boundary",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Node, is_boundary),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "connect_nodes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Node, connect_nodes),  // bytes offset in struct
    NULL,  // default value
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__size_function__Node__connect_nodes,  // size() function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_const_function__Node__connect_nodes,  // get_const(index) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_function__Node__connect_nodes,  // get(index) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__fetch_function__Node__connect_nodes,  // fetch(index, &value) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__assign_function__Node__connect_nodes,  // assign(index, value) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__resize_function__Node__connect_nodes  // resize(index) function pointer
  },
  {
    "poly_connects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Node, poly_connects),  // bytes offset in struct
    NULL,  // default value
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__size_function__Node__poly_connects,  // size() function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_const_function__Node__poly_connects,  // get_const(index) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_function__Node__poly_connects,  // get(index) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__fetch_function__Node__poly_connects,  // fetch(index, &value) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__assign_function__Node__poly_connects,  // assign(index, value) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__resize_function__Node__poly_connects  // resize(index) function pointer
  },
  {
    "contour_connects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Node, contour_connects),  // bytes offset in struct
    NULL,  // default value
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__size_function__Node__contour_connects,  // size() function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_const_function__Node__contour_connects,  // get_const(index) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_function__Node__contour_connects,  // get(index) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__fetch_function__Node__contour_connects,  // fetch(index, &value) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__assign_function__Node__contour_connects,  // assign(index, value) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__resize_function__Node__contour_connects  // resize(index) function pointer
  },
  {
    "trajectory_connects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Node, trajectory_connects),  // bytes offset in struct
    NULL,  // default value
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__size_function__Node__trajectory_connects,  // size() function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_const_function__Node__trajectory_connects,  // get_const(index) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__get_function__Node__trajectory_connects,  // get(index) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__fetch_function__Node__trajectory_connects,  // fetch(index, &value) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__assign_function__Node__trajectory_connects,  // assign(index, value) function pointer
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__resize_function__Node__trajectory_connects  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__Node_message_members = {
  "visibility_graph_msg__msg",  // message namespace
  "Node",  // message name
  13,  // number of fields
  sizeof(visibility_graph_msg__msg__Node),
  visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__Node_message_member_array,  // message members
  visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__Node_init_function,  // function to initialize message memory (memory has to be allocated)
  visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__Node_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__Node_message_type_support_handle = {
  0,
  &visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__Node_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_visibility_graph_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, visibility_graph_msg, msg, Node)() {
  visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__Node_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__Node_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__Node_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__Node_message_type_support_handle.typesupport_identifier) {
    visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__Node_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &visibility_graph_msg__msg__Node__rosidl_typesupport_introspection_c__Node_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

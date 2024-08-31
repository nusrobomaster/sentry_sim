// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from visibility_graph_msg:msg/Graph.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "visibility_graph_msg/msg/detail/graph__rosidl_typesupport_introspection_c.h"
#include "visibility_graph_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "visibility_graph_msg/msg/detail/graph__functions.h"
#include "visibility_graph_msg/msg/detail/graph__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `nodes`
#include "visibility_graph_msg/msg/node.h"
// Member `nodes`
#include "visibility_graph_msg/msg/detail/node__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  visibility_graph_msg__msg__Graph__init(message_memory);
}

void visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_fini_function(void * message_memory)
{
  visibility_graph_msg__msg__Graph__fini(message_memory);
}

size_t visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__size_function__Graph__nodes(
  const void * untyped_member)
{
  const visibility_graph_msg__msg__Node__Sequence * member =
    (const visibility_graph_msg__msg__Node__Sequence *)(untyped_member);
  return member->size;
}

const void * visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__get_const_function__Graph__nodes(
  const void * untyped_member, size_t index)
{
  const visibility_graph_msg__msg__Node__Sequence * member =
    (const visibility_graph_msg__msg__Node__Sequence *)(untyped_member);
  return &member->data[index];
}

void * visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__get_function__Graph__nodes(
  void * untyped_member, size_t index)
{
  visibility_graph_msg__msg__Node__Sequence * member =
    (visibility_graph_msg__msg__Node__Sequence *)(untyped_member);
  return &member->data[index];
}

void visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__fetch_function__Graph__nodes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const visibility_graph_msg__msg__Node * item =
    ((const visibility_graph_msg__msg__Node *)
    visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__get_const_function__Graph__nodes(untyped_member, index));
  visibility_graph_msg__msg__Node * value =
    (visibility_graph_msg__msg__Node *)(untyped_value);
  *value = *item;
}

void visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__assign_function__Graph__nodes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  visibility_graph_msg__msg__Node * item =
    ((visibility_graph_msg__msg__Node *)
    visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__get_function__Graph__nodes(untyped_member, index));
  const visibility_graph_msg__msg__Node * value =
    (const visibility_graph_msg__msg__Node *)(untyped_value);
  *item = *value;
}

bool visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__resize_function__Graph__nodes(
  void * untyped_member, size_t size)
{
  visibility_graph_msg__msg__Node__Sequence * member =
    (visibility_graph_msg__msg__Node__Sequence *)(untyped_member);
  visibility_graph_msg__msg__Node__Sequence__fini(member);
  return visibility_graph_msg__msg__Node__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Graph, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Graph, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "nodes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Graph, nodes),  // bytes offset in struct
    NULL,  // default value
    visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__size_function__Graph__nodes,  // size() function pointer
    visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__get_const_function__Graph__nodes,  // get_const(index) function pointer
    visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__get_function__Graph__nodes,  // get(index) function pointer
    visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__fetch_function__Graph__nodes,  // fetch(index, &value) function pointer
    visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__assign_function__Graph__nodes,  // assign(index, value) function pointer
    visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__resize_function__Graph__nodes  // resize(index) function pointer
  },
  {
    "size",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(visibility_graph_msg__msg__Graph, size),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_members = {
  "visibility_graph_msg__msg",  // message namespace
  "Graph",  // message name
  4,  // number of fields
  sizeof(visibility_graph_msg__msg__Graph),
  visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_member_array,  // message members
  visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_init_function,  // function to initialize message memory (memory has to be allocated)
  visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_type_support_handle = {
  0,
  &visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_visibility_graph_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, visibility_graph_msg, msg, Graph)() {
  visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, visibility_graph_msg, msg, Node)();
  if (!visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_type_support_handle.typesupport_identifier) {
    visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &visibility_graph_msg__msg__Graph__rosidl_typesupport_introspection_c__Graph_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

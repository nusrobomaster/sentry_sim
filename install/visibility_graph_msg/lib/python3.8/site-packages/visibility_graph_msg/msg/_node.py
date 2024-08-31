# generated from rosidl_generator_py/resource/_idl.py.em
# with input from visibility_graph_msg:msg/Node.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'connect_nodes'
# Member 'poly_connects'
# Member 'contour_connects'
# Member 'trajectory_connects'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Node(type):
    """Metaclass of message 'Node'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('visibility_graph_msg')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'visibility_graph_msg.msg.Node')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__node
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__node
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__node
            cls._TYPE_SUPPORT = module.type_support_msg__msg__node
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__node

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Node(metaclass=Metaclass_Node):
    """Message class 'Node'."""

    __slots__ = [
        '_header',
        '_id',
        '_freetype',
        '_position',
        '_surface_dirs',
        '_is_covered',
        '_is_frontier',
        '_is_navpoint',
        '_is_boundary',
        '_connect_nodes',
        '_poly_connects',
        '_contour_connects',
        '_trajectory_connects',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'id': 'uint32',
        'freetype': 'uint8',
        'position': 'geometry_msgs/Point',
        'surface_dirs': 'sequence<geometry_msgs/Point>',
        'is_covered': 'boolean',
        'is_frontier': 'boolean',
        'is_navpoint': 'boolean',
        'is_boundary': 'boolean',
        'connect_nodes': 'sequence<uint32>',
        'poly_connects': 'sequence<uint32>',
        'contour_connects': 'sequence<uint32>',
        'trajectory_connects': 'sequence<uint32>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point')),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('uint32')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.id = kwargs.get('id', int())
        self.freetype = kwargs.get('freetype', int())
        from geometry_msgs.msg import Point
        self.position = kwargs.get('position', Point())
        self.surface_dirs = kwargs.get('surface_dirs', [])
        self.is_covered = kwargs.get('is_covered', bool())
        self.is_frontier = kwargs.get('is_frontier', bool())
        self.is_navpoint = kwargs.get('is_navpoint', bool())
        self.is_boundary = kwargs.get('is_boundary', bool())
        self.connect_nodes = array.array('I', kwargs.get('connect_nodes', []))
        self.poly_connects = array.array('I', kwargs.get('poly_connects', []))
        self.contour_connects = array.array('I', kwargs.get('contour_connects', []))
        self.trajectory_connects = array.array('I', kwargs.get('trajectory_connects', []))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.id != other.id:
            return False
        if self.freetype != other.freetype:
            return False
        if self.position != other.position:
            return False
        if self.surface_dirs != other.surface_dirs:
            return False
        if self.is_covered != other.is_covered:
            return False
        if self.is_frontier != other.is_frontier:
            return False
        if self.is_navpoint != other.is_navpoint:
            return False
        if self.is_boundary != other.is_boundary:
            return False
        if self.connect_nodes != other.connect_nodes:
            return False
        if self.poly_connects != other.poly_connects:
            return False
        if self.contour_connects != other.contour_connects:
            return False
        if self.trajectory_connects != other.trajectory_connects:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'id' field must be an unsigned integer in [0, 4294967295]"
        self._id = value

    @builtins.property
    def freetype(self):
        """Message field 'freetype'."""
        return self._freetype

    @freetype.setter
    def freetype(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'freetype' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'freetype' field must be an unsigned integer in [0, 255]"
        self._freetype = value

    @builtins.property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'position' field must be a sub message of type 'Point'"
        self._position = value

    @builtins.property
    def surface_dirs(self):
        """Message field 'surface_dirs'."""
        return self._surface_dirs

    @surface_dirs.setter
    def surface_dirs(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'surface_dirs' field must be a set or sequence and each value of type 'Point'"
        self._surface_dirs = value

    @builtins.property
    def is_covered(self):
        """Message field 'is_covered'."""
        return self._is_covered

    @is_covered.setter
    def is_covered(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_covered' field must be of type 'bool'"
        self._is_covered = value

    @builtins.property
    def is_frontier(self):
        """Message field 'is_frontier'."""
        return self._is_frontier

    @is_frontier.setter
    def is_frontier(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_frontier' field must be of type 'bool'"
        self._is_frontier = value

    @builtins.property
    def is_navpoint(self):
        """Message field 'is_navpoint'."""
        return self._is_navpoint

    @is_navpoint.setter
    def is_navpoint(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_navpoint' field must be of type 'bool'"
        self._is_navpoint = value

    @builtins.property
    def is_boundary(self):
        """Message field 'is_boundary'."""
        return self._is_boundary

    @is_boundary.setter
    def is_boundary(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_boundary' field must be of type 'bool'"
        self._is_boundary = value

    @builtins.property
    def connect_nodes(self):
        """Message field 'connect_nodes'."""
        return self._connect_nodes

    @connect_nodes.setter
    def connect_nodes(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'I', \
                "The 'connect_nodes' array.array() must have the type code of 'I'"
            self._connect_nodes = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'connect_nodes' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._connect_nodes = array.array('I', value)

    @builtins.property
    def poly_connects(self):
        """Message field 'poly_connects'."""
        return self._poly_connects

    @poly_connects.setter
    def poly_connects(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'I', \
                "The 'poly_connects' array.array() must have the type code of 'I'"
            self._poly_connects = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'poly_connects' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._poly_connects = array.array('I', value)

    @builtins.property
    def contour_connects(self):
        """Message field 'contour_connects'."""
        return self._contour_connects

    @contour_connects.setter
    def contour_connects(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'I', \
                "The 'contour_connects' array.array() must have the type code of 'I'"
            self._contour_connects = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'contour_connects' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._contour_connects = array.array('I', value)

    @builtins.property
    def trajectory_connects(self):
        """Message field 'trajectory_connects'."""
        return self._trajectory_connects

    @trajectory_connects.setter
    def trajectory_connects(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'I', \
                "The 'trajectory_connects' array.array() must have the type code of 'I'"
            self._trajectory_connects = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'trajectory_connects' field must be a set or sequence and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._trajectory_connects = array.array('I', value)

# generated from rosidl_generator_py/resource/_idl.py.em
# with input from asv_interfaces:msg/ReferenceLlc.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ReferenceLlc(type):
    """Metaclass of message 'ReferenceLlc'."""

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
            module = import_type_support('asv_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'asv_interfaces.msg.ReferenceLlc')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__reference_llc
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__reference_llc
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__reference_llc
            cls._TYPE_SUPPORT = module.type_support_msg__msg__reference_llc
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__reference_llc

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

            from std_msgs.msg import Float64
            if Float64.__class__._TYPE_SUPPORT is None:
                Float64.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ReferenceLlc(metaclass=Metaclass_ReferenceLlc):
    """Message class 'ReferenceLlc'."""

    __slots__ = [
        '_references',
        '_u_tar',
    ]

    _fields_and_field_types = {
        'references': 'sequence<geometry_msgs/Vector3>',
        'u_tar': 'std_msgs/Float64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3')),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Float64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.references = kwargs.get('references', [])
        from std_msgs.msg import Float64
        self.u_tar = kwargs.get('u_tar', Float64())

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
        if self.references != other.references:
            return False
        if self.u_tar != other.u_tar:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def references(self):
        """Message field 'references'."""
        return self._references

    @references.setter
    def references(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
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
                 all(isinstance(v, Vector3) for v in value) and
                 True), \
                "The 'references' field must be a set or sequence and each value of type 'Vector3'"
        self._references = value

    @builtins.property
    def u_tar(self):
        """Message field 'u_tar'."""
        return self._u_tar

    @u_tar.setter
    def u_tar(self, value):
        if __debug__:
            from std_msgs.msg import Float64
            assert \
                isinstance(value, Float64), \
                "The 'u_tar' field must be a sub message of type 'Float64'"
        self._u_tar = value

# generated from rosidl_generator_py/resource/_idl.py.em
# with input from asv_interfaces:srv/SetLlc.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetLlc_Request(type):
    """Metaclass of message 'SetLlc_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'LLC_APM': 1,
        'LLC_IFAC': 2,
        'LLC_MPC': 3,
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
                'asv_interfaces.srv.SetLlc_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_llc__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_llc__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_llc__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_llc__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_llc__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'LLC_APM': cls.__constants['LLC_APM'],
            'LLC_IFAC': cls.__constants['LLC_IFAC'],
            'LLC_MPC': cls.__constants['LLC_MPC'],
        }

    @property
    def LLC_APM(self):
        """Message constant 'LLC_APM'."""
        return Metaclass_SetLlc_Request.__constants['LLC_APM']

    @property
    def LLC_IFAC(self):
        """Message constant 'LLC_IFAC'."""
        return Metaclass_SetLlc_Request.__constants['LLC_IFAC']

    @property
    def LLC_MPC(self):
        """Message constant 'LLC_MPC'."""
        return Metaclass_SetLlc_Request.__constants['LLC_MPC']


class SetLlc_Request(metaclass=Metaclass_SetLlc_Request):
    """
    Message class 'SetLlc_Request'.

    Constants:
      LLC_APM
      LLC_IFAC
      LLC_MPC
    """

    __slots__ = [
        '_llc_mode',
    ]

    _fields_and_field_types = {
        'llc_mode': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.llc_mode = kwargs.get('llc_mode', int())

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
        if self.llc_mode != other.llc_mode:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def llc_mode(self):
        """Message field 'llc_mode'."""
        return self._llc_mode

    @llc_mode.setter
    def llc_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'llc_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'llc_mode' field must be an unsigned integer in [0, 255]"
        self._llc_mode = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetLlc_Response(type):
    """Metaclass of message 'SetLlc_Response'."""

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
                'asv_interfaces.srv.SetLlc_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_llc__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_llc__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_llc__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_llc__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_llc__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetLlc_Response(metaclass=Metaclass_SetLlc_Response):
    """Message class 'SetLlc_Response'."""

    __slots__ = [
        '_success',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())

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
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value


class Metaclass_SetLlc(type):
    """Metaclass of service 'SetLlc'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('asv_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'asv_interfaces.srv.SetLlc')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_llc

            from asv_interfaces.srv import _set_llc
            if _set_llc.Metaclass_SetLlc_Request._TYPE_SUPPORT is None:
                _set_llc.Metaclass_SetLlc_Request.__import_type_support__()
            if _set_llc.Metaclass_SetLlc_Response._TYPE_SUPPORT is None:
                _set_llc.Metaclass_SetLlc_Response.__import_type_support__()


class SetLlc(metaclass=Metaclass_SetLlc):
    from asv_interfaces.srv._set_llc import SetLlc_Request as Request
    from asv_interfaces.srv._set_llc import SetLlc_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')

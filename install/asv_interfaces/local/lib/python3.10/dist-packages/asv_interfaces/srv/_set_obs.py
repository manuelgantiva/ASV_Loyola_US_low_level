# generated from rosidl_generator_py/resource/_idl.py.em
# with input from asv_interfaces:srv/SetObs.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetObs_Request(type):
    """Metaclass of message 'SetObs_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'ESO_BEJARANO': 1,
        'ESO_LIU': 2,
        'ESO_ZONO': 3,
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
                'asv_interfaces.srv.SetObs_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_obs__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_obs__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_obs__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_obs__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_obs__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ESO_BEJARANO': cls.__constants['ESO_BEJARANO'],
            'ESO_LIU': cls.__constants['ESO_LIU'],
            'ESO_ZONO': cls.__constants['ESO_ZONO'],
        }

    @property
    def ESO_BEJARANO(self):
        """Message constant 'ESO_BEJARANO'."""
        return Metaclass_SetObs_Request.__constants['ESO_BEJARANO']

    @property
    def ESO_LIU(self):
        """Message constant 'ESO_LIU'."""
        return Metaclass_SetObs_Request.__constants['ESO_LIU']

    @property
    def ESO_ZONO(self):
        """Message constant 'ESO_ZONO'."""
        return Metaclass_SetObs_Request.__constants['ESO_ZONO']


class SetObs_Request(metaclass=Metaclass_SetObs_Request):
    """
    Message class 'SetObs_Request'.

    Constants:
      ESO_BEJARANO
      ESO_LIU
      ESO_ZONO
    """

    __slots__ = [
        '_eso_mode',
    ]

    _fields_and_field_types = {
        'eso_mode': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.eso_mode = kwargs.get('eso_mode', int())

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
        if self.eso_mode != other.eso_mode:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def eso_mode(self):
        """Message field 'eso_mode'."""
        return self._eso_mode

    @eso_mode.setter
    def eso_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'eso_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'eso_mode' field must be an unsigned integer in [0, 255]"
        self._eso_mode = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetObs_Response(type):
    """Metaclass of message 'SetObs_Response'."""

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
                'asv_interfaces.srv.SetObs_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_obs__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_obs__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_obs__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_obs__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_obs__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetObs_Response(metaclass=Metaclass_SetObs_Response):
    """Message class 'SetObs_Response'."""

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


class Metaclass_SetObs(type):
    """Metaclass of service 'SetObs'."""

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
                'asv_interfaces.srv.SetObs')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_obs

            from asv_interfaces.srv import _set_obs
            if _set_obs.Metaclass_SetObs_Request._TYPE_SUPPORT is None:
                _set_obs.Metaclass_SetObs_Request.__import_type_support__()
            if _set_obs.Metaclass_SetObs_Response._TYPE_SUPPORT is None:
                _set_obs.Metaclass_SetObs_Response.__import_type_support__()


class SetObs(metaclass=Metaclass_SetObs):
    from asv_interfaces.srv._set_obs import SetObs_Request as Request
    from asv_interfaces.srv._set_obs import SetObs_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')

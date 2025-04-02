# generated from rosidl_generator_py/resource/_idl.py.em
# with input from tros_ai_fusion_msgs:srv/TopicManage.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TopicManage_Request(type):
    """Metaclass of message 'TopicManage_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'ADD': 'add',
        'DELETE': 'delete',
        'GET': 'get',
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('tros_ai_fusion_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tros_ai_fusion_msgs.srv.TopicManage_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__topic_manage__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__topic_manage__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__topic_manage__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__topic_manage__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__topic_manage__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ADD': cls.__constants['ADD'],
            'DELETE': cls.__constants['DELETE'],
            'GET': cls.__constants['GET'],
        }

    @property
    def ADD(self):
        """Message constant 'ADD'."""
        return Metaclass_TopicManage_Request.__constants['ADD']

    @property
    def DELETE(self):
        """Message constant 'DELETE'."""
        return Metaclass_TopicManage_Request.__constants['DELETE']

    @property
    def GET(self):
        """Message constant 'GET'."""
        return Metaclass_TopicManage_Request.__constants['GET']


class TopicManage_Request(metaclass=Metaclass_TopicManage_Request):
    """
    Message class 'TopicManage_Request'.

    Constants:
      ADD
      DELETE
      GET
    """

    __slots__ = [
        '_action',
        '_topics',
    ]

    _fields_and_field_types = {
        'action': 'string',
        'topics': 'sequence<string>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.action = kwargs.get('action', str())
        self.topics = kwargs.get('topics', [])

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
        if self.action != other.action:
            return False
        if self.topics != other.topics:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def action(self):
        """Message field 'action'."""
        return self._action

    @action.setter
    def action(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'action' field must be of type 'str'"
        self._action = value

    @builtins.property
    def topics(self):
        """Message field 'topics'."""
        return self._topics

    @topics.setter
    def topics(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'topics' field must be a set or sequence and each value of type 'str'"
        self._topics = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_TopicManage_Response(type):
    """Metaclass of message 'TopicManage_Response'."""

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
            module = import_type_support('tros_ai_fusion_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tros_ai_fusion_msgs.srv.TopicManage_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__topic_manage__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__topic_manage__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__topic_manage__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__topic_manage__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__topic_manage__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TopicManage_Response(metaclass=Metaclass_TopicManage_Response):
    """Message class 'TopicManage_Response'."""

    __slots__ = [
        '_result',
        '_topics',
    ]

    _fields_and_field_types = {
        'result': 'boolean',
        'topics': 'sequence<string>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.result = kwargs.get('result', bool())
        self.topics = kwargs.get('topics', [])

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
        if self.result != other.result:
            return False
        if self.topics != other.topics:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def result(self):
        """Message field 'result'."""
        return self._result

    @result.setter
    def result(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'result' field must be of type 'bool'"
        self._result = value

    @builtins.property
    def topics(self):
        """Message field 'topics'."""
        return self._topics

    @topics.setter
    def topics(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'topics' field must be a set or sequence and each value of type 'str'"
        self._topics = value


class Metaclass_TopicManage(type):
    """Metaclass of service 'TopicManage'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('tros_ai_fusion_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'tros_ai_fusion_msgs.srv.TopicManage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__topic_manage

            from tros_ai_fusion_msgs.srv import _topic_manage
            if _topic_manage.Metaclass_TopicManage_Request._TYPE_SUPPORT is None:
                _topic_manage.Metaclass_TopicManage_Request.__import_type_support__()
            if _topic_manage.Metaclass_TopicManage_Response._TYPE_SUPPORT is None:
                _topic_manage.Metaclass_TopicManage_Response.__import_type_support__()


class TopicManage(metaclass=Metaclass_TopicManage):
    from tros_ai_fusion_msgs.srv._topic_manage import TopicManage_Request as Request
    from tros_ai_fusion_msgs.srv._topic_manage import TopicManage_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')

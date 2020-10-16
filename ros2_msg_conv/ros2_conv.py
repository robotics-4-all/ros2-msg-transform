from collections import OrderedDict

from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.import_message import import_message_from_namespaced_type
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_parser.definition import NamespacedType


def ros2_msg_to_dict(msg):
    _d = message_to_ordereddict(msg)
    return _d


def dict_to_ros2_msg(py_dict, msg_cls):
    m = msg_cls()
    set_message_fields(m, py_dict)
    return m


def dict_to_ros2_msg_from_ns(py_dict, msg_type):
    (pkg, name) = msg_type.split('/')
    msg_ns_type = NamespacedType([pkg, 'msg'], name)
    msg_cls = import_message_from_namespaced_type(msg_ns_type)
    print(msg_cls)
    return dict_to_ros2_msg(py_dict, msg_cls)


def dict_to_ros2_srv_from_ns(py_dict, srv_type):
    if '.' not in srv_type:
        raise ValueError('Not a valid srv_type given.')
    (pkg, name) = srv_type.split('/')
    msg_ns_type = NamespacedType([pkg, 'srv'], name.split('.')[0])
    msg_cls = import_message_from_namespaced_type(msg_ns_type)
    if name.split('.')[1] == 'Request':
        return dict_to_ros2_msg(py_dict, msg_cls.Request)
    elif name.split('.')[1] == 'Response':
        return dict_to_ros2_msg(py_dict, msg_cls.Response)

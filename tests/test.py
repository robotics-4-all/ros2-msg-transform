from std_msgs.msg import String
from std_srvs.srv import SetBool
from sensor_msgs.msg import Range
import json

from ros2_msg_conv import (
    ros2_msg_to_dict, dict_to_ros2_msg_from_ns, dict_to_ros2_msg,
    dict_to_ros2_srv_from_ns
)


def test_msg():
    msg = Range()
    _dict = ros2_msg_to_dict(msg)
    print(_dict)
    _msg = dict_to_ros2_msg(_dict, Range)
    print(_msg)
    _msg = dict_to_ros2_msg_from_ns(_dict, 'sensor_msgs/Range')
    print(_msg)


def test_srv():
    print('[*] - Testing ROS2 Service Request to Dict (OrderedDict)')
    req = SetBool.Request()
    _dict = ros2_msg_to_dict(req)
    print(_dict)
    _msg = dict_to_ros2_msg(_dict, SetBool.Request)
    print(_msg)
    _msg = dict_to_ros2_srv_from_ns(_dict, 'std_srvs/SetBool.Request')
    print(_msg)
    print('[*] - Testing ROS2 Service Response to Dict (OrderedDict)')
    req = SetBool.Response()
    _dict = ros2_msg_to_dict(req)
    print(_dict)
    _msg = dict_to_ros2_msg(_dict, SetBool.Response)
    print(_msg)


if __name__ == '__main__':
    test_srv()

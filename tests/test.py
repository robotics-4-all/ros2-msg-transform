#!/usr/bin/env python3

from std_msgs.msg import String
from std_srvs.srv import SetBool
from sensor_msgs.msg import Range
import json

from ros2_msg_transform import (
    ros2_msg_to_dict, dict_to_ros2_msg_from_ns, dict_to_ros2_msg,
    dict_to_ros2_srv_from_ns
)


def test_msg():
    print('[*] - Testing ROS2 Message Conversions')
    msg = Range()
    _dict = ros2_msg_to_dict(msg)
    print(_dict)
    _msg = dict_to_ros2_msg(_dict, Range)
    print(_msg)
    _msg = dict_to_ros2_msg_from_ns(_dict, 'sensor_msgs/Range')
    print(_msg)


def test_srv():
    print()
    print('[*] - Testing ROS2 Service Request Conversions')
    req = SetBool.Request()
    _dict = ros2_msg_to_dict(req)
    print(_dict)
    _msg = dict_to_ros2_msg(_dict, SetBool.Request)
    print(_msg)
    _msg = dict_to_ros2_srv_from_ns(_dict, 'std_srvs/SetBool.Request')
    print(_msg)
    print()
    print('[*] - Testing ROS2 Service Response Conversions')
    req = SetBool.Response()
    _dict = ros2_msg_to_dict(req)
    print(_dict)
    _msg = dict_to_ros2_msg(_dict, SetBool.Response)
    print(_msg)


if __name__ == '__main__':
    test_msg()
    test_srv()

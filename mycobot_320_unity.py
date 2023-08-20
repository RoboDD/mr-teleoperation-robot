#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""[summary]
This file obtains the joint angle of the manipulator in ROS,
and then sends it directly to the real manipulator using `pymycobot` API.
This file is [slider_control.launch] related script.
Passable parameters:
    port: serial prot string. Defaults is '/dev/ttyAMA0'
    baud: serial prot baudrate. Defaults is 1000000.
"""

import rospy
from sensor_msgs.msg import JointState

from pymycobot.mycobot import MyCobot
from mycobot_communication.srv import GetCoords, SetCoords, GetAngles, SetAngles, GripperStatus

mc = None


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "%s", data.position)
    print(data.position)
    data_list = []
    for index, value in enumerate(data.position):
        data_list.append(value)

    mc.send_radians(data_list, 80)

    gripper_cmd=data.effort[5]

    if gripper_cmd > 0.5:
        mc.set_gripper_state(1,50)
    else:
        mc.set_gripper_state(0,50)

    # time.sleep(0.5)


def listener():
    global mc
    rospy.init_node("control_slider", anonymous=True)

    rospy.Subscriber("joint_states", JointState, callback)
    


    port = rospy.get_param("~port", "/dev/ttyAMA0")
    baud = rospy.get_param("~baud", 115200)
    print(port, baud)
    mc = MyCobot(port, baud)



    # spin() simply keeps python from exiting until this node is stopped
    print("spin ...")
    rospy.spin()


if __name__ == "__main__":
    listener()
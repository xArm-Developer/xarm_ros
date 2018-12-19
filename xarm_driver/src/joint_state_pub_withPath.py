#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFactory, Inc.
# All rights reserved.
#
# Author: Jason PENG <jason@ufactory.cc>

# This is the joint state feedback process of real xArm7
import sys
import rospy
import copy
import argparse
import tf
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
# add python library path to system path
# sys.path.append("/Directory/to/the/xArm-Python-SDK/")
from xarm.wrapper import XArmAPI

PUB_TOPIC = 'joint_states'
PUB_RATE_HZ = 20
DOF = 7
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']\


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', dest='ip', default="192.168.1.11", help="your robot controler ip")
    parser.add_argument('__name:', help="ROS Node Name")
    parser.add_argument('__log:', help="ROS Node log file Name")

    args = parser.parse_args()

    # connect to the real xArm7 hardware with specified IP address 
    xarm = XArmAPI(args.ip, do_not_open=False)

    js_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('jnt_state_pub', anonymous=True)
    rate = rospy.Rate(PUB_RATE_HZ) # 10hz
    msg = JointState()
    msg.name = JOINT_NAMES
    now = rospy.get_rostime()
    prev_position = [0]*DOF
    msg.position = [0]*DOF
    msg.effort = [0]*DOF

    listener = tf.TransformListener()
    path_pub = rospy.Publisher('tip_path', Path, queue_size=10)
    msg2 = Path()
    stamped_pose = PoseStamped()

    # TODO: check if /use_sim_time is set false
    while not rospy.is_shutdown():
        # TODO: get servo feedback and set the JointState msg
        prev_position = copy.deepcopy(msg.position)
        now = rospy.get_rostime()
        msg.header.stamp.secs = now.secs
        msg.header.stamp.nsecs = now.nsecs

        msg.position = xarm.get_servo_angle()[1] # is_radian true by default

        msg.velocity = [ ((msg.position[i] - prev_position[i])*PUB_RATE_HZ) for i in range(DOF)]

        js_pub.publish(msg)

        try:
            (trans,rot) = listener.lookupTransform('/link7', '/link_base', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        msg2.header = msg.header # deepcopy?
        stamped_pose.header = msg.header # deepcopy?
        stamped_pose.pose.position.x = trans[0]
        stamped_pose.pose.position.y = trans[1]
        stamped_pose.pose.position.z = trans[2]
        stamped_pose.pose.orientation.x = rot[0]
        stamped_pose.pose.orientation.y = rot[1]
        stamped_pose.pose.orientation.z = rot[2]
        stamped_pose.pose.orientation.w = rot[3]
        msg2.poses.append(stamped_pose)

        rate.sleep()

    xarm.disconnect()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

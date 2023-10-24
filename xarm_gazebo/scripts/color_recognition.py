#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import sys
import rospy
import math
import random
import threading
from distutils.version import LooseVersion
import numpy as np
import moveit_commander
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

if sys.version_info < (3, 0):
    PY3 = False
    import Queue as queue
else:
    PY3 = True
    import queue

boxPoints = cv2.boxPoints if LooseVersion(cv2.__version__) >= LooseVersion('3.0') else cv2.cv.BoxPoints

COLOR_DICT = {
    'red': {'lower': np.array([0, 43, 46]), 'upper': np.array([10, 255, 255])},
    'blue': {'lower': np.array([90, 100, 100]), 'upper': np.array([130, 255, 255])},
    'green': {'lower': np.array([50, 60, 60]), 'upper': np.array([77, 255, 255])},
    'yellow': {'lower': np.array([20, 40, 46]), 'upper': np.array([34, 255, 255])},
}


class GripperCtrl(object):
    def __init__(self):
        self._commander = moveit_commander.move_group.MoveGroupCommander('xarm_gripper')
        self._init()

    def _init(self):
        self._commander.set_max_acceleration_scaling_factor(1.0)
        self._commander.set_max_velocity_scaling_factor(1.0)
    
    def open(self, wait=True):
        try:
            self._commander.set_named_target('open')
            ret = self._commander.go(wait=wait)
            print('gripper_open, ret={}'.format(ret))
            return ret
        except Exception as e:
            print('[Ex] gripper open exception, {}'.format(e))
        return False

    def close(self, wait=True):
        try:
            self._commander.set_named_target('close')
            ret = self._commander.go(wait=wait)
            print('gripper_close, ret={}'.format(ret))
            return ret
        except Exception as e:
            print('[Ex] gripper close exception, {}'.format(e))
        return False


class XArmCtrl(object):
    def __init__(self, dof):
        self._commander = moveit_commander.move_group.MoveGroupCommander('xarm{}'.format(dof))
        self.dof = int(dof)
        self._init()
    
    def _init(self):
        self._commander.set_max_acceleration_scaling_factor(1.0)
        self._commander.set_max_velocity_scaling_factor(1.0)

    def set_joints(self, angles, wait=True):
        try:
            joint_target = self._commander.get_current_joint_values()
            for i in range(joint_target):
                if i >= len(angles):
                    break
                if angles[i] is not None:
                    joint_target[i] = math.radians(angles[i])
            print('set_joints, joints={}'.format(joint_target))
            self._commander.set_joint_value_target(joint_target)
            ret = self._commander.go(wait=wait)
            print('move to finish, ret={}'.format(ret))
            return ret
        except Exception as e:
            print('[Ex] set_joints exception, ex={}'.format(e))
    
    def set_joint(self, angle, inx=-1, wait=True):
        try:
            joint_target = self._commander.get_current_joint_values()
            joint_target[inx] = math.radians(angle)
            print('set_joints, joints={}'.format(joint_target))
            self._commander.set_joint_value_target(joint_target)
            ret = self._commander.go(wait=wait)
            print('move to finish, ret={}'.format(ret))
            return ret
        except Exception as e:
            print('[Ex] set_joint exception, ex={}'.format(e))
        return False

    def moveto(self, x=None, y=None, z=None, ox=None, oy=None, oz=None, relative=False, wait=True):
        if x == 0 and y == 0 and z == 0 and ox == 0 and oy == 0 and oz == 0 and relative:
            return True
        try:
            pose_target = self._commander.get_current_pose().pose
            if relative:
                pose_target.position.x += x / 1000.0 if x is not None else 0
                pose_target.position.y += y / 1000.0 if y is not None else 0
                pose_target.position.z += z / 1000.0 if z is not None else 0
                pose_target.orientation.x += ox if ox is not None else 0
                pose_target.orientation.y += oy if oy is not None else 0
                pose_target.orientation.z += oz if oz is not None else 0
            else:
                pose_target.position.x = x / 1000.0 if x is not None else pose_target.position.x
                pose_target.position.y = y / 1000.0 if y is not None else pose_target.position.y
                pose_target.position.z = z / 1000.0 if z is not None else pose_target.position.z
                pose_target.orientation.x = ox if ox is not None else pose_target.orientation.x
                pose_target.orientation.y = oy if oy is not None else pose_target.orientation.y
                pose_target.orientation.z = oz if oz is not None else pose_target.orientation.z
            print('move to position=[{:.2f}, {:.2f}, {:.2f}], orientation=[{:.6f}, {:.6f}, {:.6f}]'.format(
                pose_target.position.x * 1000.0, pose_target.position.y * 1000.0, pose_target.position.z * 1000.0,
                pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z
            ))
            if self.dof == 7:
                path, fraction = self._commander.compute_cartesian_path([pose_target], 0.005, 0.0)
                if fraction < 0.9:
                    ret = False
                else:
                    ret = self._commander.execute(path, wait=wait)
                print('move to finish, ret={}, fraction={}'.format(ret, fraction))
            else:
                self._commander.set_pose_target(pose_target)
                ret = self._commander.go(wait=wait)
                print('move to finish, ret={}'.format(ret))
            return ret
        except Exception as e:
            print('[Ex] moveto exception: {}'.format(e))
        return False


class GazeboMotionThread(threading.Thread):
    def __init__(self, que, **kwargs):
        if PY3:
            super().__init__()
        else:
            super(GazeboMotionThread, self).__init__()
        self.que = que
        self.daemon = True
        self.in_motion = True
        dof = kwargs.get('dof', 6)
        self._xarm_ctrl = XArmCtrl(dof)
        self._gripper_ctrl = GripperCtrl()
        self._grab_z = kwargs.get('grab_z', 10)
        self._safe_z = kwargs.get('safe_z', 100)
    
    @staticmethod
    def _rect_to_move_params(rect):
        return int((466 - rect[0][1]) * 900.0 / 460.0 + 253.3), int((552 - rect[0][0]) * 900.0 / 460.0 - 450), rect[2] - 90

    def run(self):
        while True:
            # self._xarm_ctrl.set_joint(0)
            self._gripper_ctrl.open()
            if not self._xarm_ctrl.moveto(z=self._safe_z):
                continue
            if not self._xarm_ctrl.moveto(x=300, y=0, z=self._safe_z):
                continue
            self.in_motion = False
            rects = self.que.get()
            self.in_motion = True
            
            rect = rects[random.randint(0, 100) % len(rects)]
            x, y, angle = self._rect_to_move_params(rect)
            print('target: x={:.2f}mm, y={:.2f}mm, anlge={:.2f}'.format(x, y, angle))
            ret = self._xarm_ctrl.moveto(z=self._safe_z)
            if not ret:
                continue
            ret = self._xarm_ctrl.moveto(x=x, y=y, z=self._safe_z)
            if not ret:
                continue
            # ret = self._xarm_ctrl.set_joint(angle)
            # if not ret:
            #     continue
            ret = self._xarm_ctrl.moveto(x=x, y=y, z=self._grab_z)
            if not ret:
                continue
            self._gripper_ctrl.close()
            ret = self._xarm_ctrl.moveto(x=x, y=y, z=self._safe_z)
            if not ret:
                continue
            ret = self._xarm_ctrl.moveto(x=x, y=y, z=self._grab_z)
            if not ret:
                continue
            self._gripper_ctrl.open()
            self._xarm_ctrl.moveto(x=x, y=y, z=self._safe_z)


class GazeboCamera(object):
    def __init__(self, topic_name='/camera/image_raw/compressed'):
        self._frame_que = queue.Queue(10)
        self._bridge = CvBridge()
        self._img_sub = rospy.Subscriber(topic_name, CompressedImage, self._img_callback)

    def _img_callback(self, data):
        if self._frame_que.full():
            self._frame_que.get()
        self._frame_que.put(self._bridge.compressed_imgmsg_to_cv2(data))
    
    def get_frame(self):
        if self._frame_que.empty():
            return None
        return self._frame_que.get()


def get_recognition_rect(frame, lower=COLOR_DICT['red']['lower'], upper=COLOR_DICT['red']['upper'], show=True):
    gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
    erode_hsv = cv2.erode(hsv, None, iterations=2)
    inRange_hsv = cv2.inRange(erode_hsv, lower, upper)
    contours = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    rects = []
    for _, c in enumerate(contours):
        rect = cv2.minAreaRect(c)
        if rect[1][0] < 20 or rect[1][1] < 20:
            continue
        # print(rect)
        box = boxPoints(rect)
        cv2.drawContours(frame, [np.int0(box)], -1, (0, 255, 255), 1)
        rects.append(rect)
    
    if show:
        cv2.imshow("Frame", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown('key to exit')
    return rects


if __name__ == '__main__':
    rospy.init_node('color_recognition_node', anonymous=False)
    dof = rospy.get_param('/xarm/DOF', default=6)
    rate = rospy.Rate(10.0)

    motion_que = queue.Queue(1)
    motion = GazeboMotionThread(motion_que, dof=dof)
    motion.start()

    color = COLOR_DICT['red']

    cam = GazeboCamera(topic_name='/camera/image_raw/compressed')

    while not rospy.is_shutdown():
        rate.sleep()
        frame = cam.get_frame()
        if frame is None:
            continue
        rects = get_recognition_rect(frame, lower=color['lower'], upper=color['upper'])
        if len(rects) == 0:
            continue
        if motion.in_motion or motion_que.qsize() != 0:
            continue
        motion_que.put(rects)
        
    
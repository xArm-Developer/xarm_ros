import cv2
import rospy
import math
import queue
import random
import threading
import numpy as np
import moveit_commander
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image

COLOR_DICT = {
    'red': {'lower': np.array([0, 60, 60]), 'upper': np.array([6, 255, 255])},
    'blue': {'lower': np.array([100, 80, 46]), 'upper': np.array([124, 255, 255])},
    'green': {'lower': np.array([35, 43, 35]), 'upper': np.array([90, 255, 255])},
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
            return self._commander.go(wait=wait)
        except Exception as e:
            print('gripper open exception, {}'.format(e))
        return -1

    def close(self, wait=True):
        try:
            self._commander.set_named_target('close')
            return self._commander.go(wait=wait)
        except Exception as e:
            print('gripper close exception, {}'.format(e))
        return -1


class XArmCtrl(object):
    def __init__(self, dof):
        self._commander = moveit_commander.move_group.MoveGroupCommander('xarm{}'.format(dof))
        self._init()
    
    def _init(self):
        self._commander.set_max_acceleration_scaling_factor(1.0)
        self._commander.set_max_velocity_scaling_factor(1.0)

    def rotation_end(self, angle, wait=True):
        try:
            joint_target = self._commander.get_current_joint_values()
            joint_target[-1] = math.radians(angle - 90)
            self._commander.set_joint_value_target(joint_target)
            return self._commander.go(wait=wait)
        except Exception as e:
            print('rotation end exception, {}'.format(e))
        return -1

    def moveto(self, x=None, y=None, z=None, relative=True, wait=True):
        if x == 0 and y == 0 and z == 0 and relative:
            return 0
        try:
            pose_target = self._commander.get_current_pose().pose
            if relative:
                pose_target.position.x += x if x is not None else 0
                pose_target.position.y += y if y is not None else 0
                pose_target.position.z += z if z is not None else 0
            else:
                pose_target.position.x = x if x is not None else pose_target.position.x
                pose_target.position.y = y if y is not None else pose_target.position.y
                pose_target.position.z = z if z is not None else pose_target.position.z
            print('move to [{}, {}, {}]'.format(
                pose_target.position.x, pose_target.position.y, pose_target.position.z
            ))
            self._commander.set_pose_target(pose_target)
            self._commander.go(wait=wait)
            print('move to finish')
            return 0
        except Exception as e:
            print('moveto exception: {}'.format(e))
        return -1


class MotionThread(threading.Thread):
    def __init__(self, que, dof=6):
        super().__init__()
        self.que = que
        self.daemon = True
        self.in_motion = True
        self._xarm_ctrl = XArmCtrl(dof)
        self._gripper_ctrl = GripperCtrl()

    @staticmethod
    def rect_to_xarm_xy(rect):
        return int((466 - rect[0][1]) * 900 / 460 + 253.3), int((552 - rect[0][0]) * 900 / 460 - 450), rect[2]

    def run(self):
        self._xarm_ctrl.rotation_end(90)
        self._xarm_ctrl.moveto(z=0.08, relative=False)
        self._gripper_ctrl.open()
        self.in_motion = False
        while True:
            rect = self.que.get()
            self.in_motion = True
            x, y, angle = self.rect_to_xarm_xy(rect)
            print('target: x={}mm, y={}mm, anlge={}'.format(x, y, angle))
            self._xarm_ctrl.moveto(z=0.08, relative=False)
            self._xarm_ctrl.rotation_end(angle)
            self._xarm_ctrl.moveto(x=x / 1000, y=y / 1000, relative=False)
            self._gripper_ctrl.open()

            self._xarm_ctrl.moveto(z=-0.07, relative=True)
            self._gripper_ctrl.close()

            self._xarm_ctrl.moveto(z=0.1, relative=True)
            self._gripper_ctrl.open()

            self._xarm_ctrl.moveto(x=0.3, y=0, z=0.08, relative=False)
            # self._xarm_ctrl.rotation_end(90)

            self.in_motion = False


class ColorRecognition(object):
    def __init__(self, dof):
        self._color = COLOR_DICT['red']
        self._que = queue.Queue(1)
        self._bridge = CvBridge()
        self._motion = MotionThread(self._que, dof=dof)
        self._motion.start()
        # self._img_sub = rospy.Subscriber('/kinect_V2/rgb/image_raw/compressed', CompressedImage, self._img_callback)
        self._img_sub = rospy.Subscriber('/camera/image_raw/compressed', CompressedImage, self._img_callback)
    
    def _img_callback(self, data):
        frame = self._bridge.compressed_imgmsg_to_cv2(data)
        gs_frame = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(gs_frame, cv2.COLOR_BGR2HSV)
        erode_hsv = cv2.erode(hsv, None, iterations=2)
        inRange_hsv = cv2.inRange(erode_hsv, self._color['lower'], self._color['upper'])
        contours = cv2.findContours(inRange_hsv.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        cnts = len(contours)
        inx = random.randint(0, 100) % cnts
        for i, c in enumerate(contours):
            rect = cv2.minAreaRect(c)
            if rect[1][0] < 20 or rect[1][1] < 20:
                continue
            box = cv2.boxPoints(rect)
            cv2.drawContours(frame, [np.int0(box)], -1, (0, 255, 255), 1)
            if inx == i and not self._motion.in_motion and self._que.qsize() == 0:
                self._que.put(rect)

        cv2.imshow("Frame", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.signal_shutdown('key to exit')


if __name__ == '__main__':
    rospy.init_node('color_recognition_node', anonymous=False)
    dof = rospy.get_param('/xarm/DOF', default=6)
    color_reg = ColorRecognition(dof)
    rospy.loginfo('Color Recognition Node Started')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')
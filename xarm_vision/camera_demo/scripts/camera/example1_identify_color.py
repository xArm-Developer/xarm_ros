#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# Copyright 2019 The UFACTORY Inc. All Rights Reserved.
#
# Software License Agreement (BSD License)
#
# Author: Jimy Zhang <jimy.zhang@ufactory.cc> <jimy92@163.com>
# =============================================================================


#import rospy
#from std_msgs.msg import String
from camera_driver import CameraDriver

def main():
  #实例化CameraDriver，参数0 (调用/dev/video0摄像头)
  camera = CameraDriver(0)
  pose_mid = [[0,0]]*10
  s = [0]*10
  while(1):
    image = camera.get_image()  # 获取一帧图片

    # 识别image中的蓝色物体
    camera.identify_colour(image, pose_mid, s, 1) # 识别image中的蓝色物体
    # 打印蓝色物体的坐标信息
    print(pose_mid)
    # 打印蓝色物品的面积
    print(s)
    print(' ')

if __name__ == '__main__':
  main()

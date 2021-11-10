#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# Copyright 2019 The UFACTORY Inc. All Rights Reserved.
#
# Software License Agreement (BSD License)
#
# Author: Jimy Zhang <jimy.zhang@ufactory.cc> <jimy92@163.com>
# =============================================================================

import cv2
import numpy as np
import math
import time

class CameraDriver(object):
  def __init__(self, video_port):
    self.cameraCapture = cv2.VideoCapture(video_port)
    self.xy2=[1,1]
    self.state = 0

  def close(self):
    cv2.destroyAllWindows()

  def get_image(self):
    cv2.waitKey(1)
    success, image_src = self.cameraCapture.read()
    if success:
      return image_src
    else:
      print("ERROR: cameraCapture.read()")
      self.state = -1

  # 识别image中的蓝色物体
  # pose_mid：蓝色物体的像素坐标
  # s：蓝色物体的体积
  # show_image：1->显示过程图片；0->不显示过程图片
  def identify_colour(self, image, pose_mid, s, show_image=0):
    if self.state == -1:
      return

    image2_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    LowerBlue = np.array([90, 100, 100])
    UpperBlue = np.array([130, 255, 255])
    mask = cv2.inRange(image2_hsv, LowerBlue, UpperBlue)
    image3_hsv = cv2.bitwise_and(image2_hsv, image2_hsv, mask=mask)
    image4_gry = image3_hsv[:,:,0]

    blurred = cv2.blur(image4_gry, (9, 9))
    (_, thresh) = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (25, 25))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    closed = cv2.erode(closed, None, iterations=4)
    closed = cv2.dilate(closed, None, iterations=4)
    if show_image == 1:
      cv2.imshow('win6_bin', closed)

    aaa, contours, hier = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    i = 0
    for c in contours:
      x, y, w, h = cv2.boundingRect(c)
      rect = cv2.minAreaRect(c)
      box = cv2.boxPoints(rect)
      box =np.int0(box)
      cv2.drawContours(image, [box], 0, (0, 0, 255), 3)

      pose_mid[i] = [(box[0][0] + box[2][0])/2, (box[0][1] + box[2][1])/2]
      w = math.sqrt((box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2)
      h = math.sqrt((box[0][0] - box[3][0])**2 + (box[0][1] - box[3][1])**2)
      s[i] = w * h
      s.sort(reverse=True)
      i = i + 1

    if show_image == 1:
      cv2.imshow("Image", image)

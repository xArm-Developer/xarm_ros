#!/usr/bin/env python

# run example: $ rosrun xarm_api blended_motion_test.py 6

import sys
import time
import rospy
from xarm_msgs.srv import *

def blended_motions(axis_num):
	move_lineb = rospy.ServiceProxy('/xarm/move_lineb', Move)
	move_jointb = rospy.ServiceProxy('/xarm/move_jointb', Move)
	req_lineb = MoveRequest() 
	req_lineb.mvvelo = 100 # mm/s
	req_lineb.mvacc = 1000 # mm/s^2
	req_lineb.mvtime = 0 
	req_lineb.mvradii = 25 # blending radius: 25 mm
	position_list = [[300, 200, 100, -3.14, 0, 0], [500, 200, 100, -3.14, 0, 0], [500, -200, 100, -3.14, 0, 0], [300, -200, 100, -3.14, 0, 0]]
	
	req_jointb = MoveRequest() 
	req_jointb.mvvelo = 0.4 # rad/s
	req_jointb.mvacc = 30 # rad/s^2
	req_jointb.mvtime = 0 
	req_jointb.mvradii = 25 # blending radius: 25 mm

	ret = 0
	try:
		# blended linear motions: 
		for i in range(len(position_list)):
			req_lineb.pose = position_list[i]
			res = move_lineb(req_lineb)
			if res.ret:
				print("Something Wrong happened calling move_lineb service, index is %d, ret = %d"%(i, res.ret))
				ret = -1
				break
		# followed by a blended joint motion: 
		if ret == 0:
			req_jointb.pose = [0]*axis_num # zero joint position
			res = move_jointb(req_jointb)
			if res.ret:
				print("Something Wrong happened calling move_jointb service, ret = %d"%(res.ret,))
				ret = -1

		return ret

	except rospy.ServiceException as e:
		print("motion Service call failed: %s"%e)
		return -1


if __name__ == "__main__":

	if not len(sys.argv) == 2:
		print('Please specify xArm Degree of Freedom (5/6/7) !')
		exit(-1)

	dof = int(sys.argv[1])
	
	rospy.wait_for_service('/xarm/move_jointb')
	rospy.set_param('/xarm/wait_for_finish', True) # return after motion service finish
	
	motion_en = rospy.ServiceProxy('/xarm/motion_ctrl', SetAxis)
	set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
	set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)
	get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
	home = rospy.ServiceProxy('/xarm/go_home', Move)

	try:
		motion_en(8,1)
		set_mode(0)
		set_state(0)
		req = MoveRequest() # MoveRequest for go_home
		req.mvvelo = 0.5 # rad/s
		req.mvacc = 30 # rad/s^2
		req.mvtime = 0
		home(req)

	except rospy.ServiceException as e:
		print("go_home, service call failed: %s"%e)
		exit(-1)

	rospy.set_param('/xarm/wait_for_finish', False) # *CRITICAL for move_lineb and move_jointb, for successful blending

	if blended_motions(dof) == 0:
		print("execution finished!")

	rospy.set_param('/xarm/wait_for_finish', True) # After sending all blending motion commands, you can set wait_for_finish back to true if needed.

#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFactory, Inc.
# All rights reserved.
#
# Author: Jason PENG <jason@ufactory.cc>

import rospy
import actionlib
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
import threading
import sys
import argparse
import copy
import socket

# add xArm python library path to system path
# sys.path.append("/Directory/to/the/xArm-Python-SDK/")
from xarm.wrapper import XArmAPI

DOF = 7
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
UPDATE_RATE_HZ = 100
STOP_DURATION = 0.1
P_TOLERANCE = [0.01]*DOF
GAP_TOL_RAD = 0.1
EXEC_TIME_TOLERANCE_SEC = 2.0

# global joint state variable
curr_jpose = [0]*DOF
jpose_lock = threading.Lock()


def reorder_traj_joints(traj, joint_names):
    order = [traj.joint_names.index(j) for j in joint_names]

    new_points = []
    for p in traj.points:
        new_points.append(JointTrajectoryPoint(
            positions = [p.positions[i] for i in order],
            velocities = [p.velocities[i] for i in order] if p.velocities else [],
            accelerations = [p.accelerations[i] for i in order] if p.accelerations else [],
            time_from_start = p.time_from_start))
    traj.joint_names = joint_names
    traj.points = new_points

def cubic_spline_interp(p0, p1, t_abs):
	# jason comment: Cubic interpolation between adjacent waypoints
    T = (p1.time_from_start - p0.time_from_start).to_sec()
    t = t_abs - p0.time_from_start.to_sec()
    q = [0] * DOF
    qdot = [0] * DOF
    qddot = [0] * DOF
    # jason comment: i in range(DOF)
    for i in range(len(p0.positions)):
        a = p0.positions[i]
        b = p0.velocities[i]
        c = (-3*p0.positions[i] + 3*p1.positions[i] - 2*T*p0.velocities[i] - T*p1.velocities[i]) / T**2
        d = (2*p0.positions[i] - 2*p1.positions[i] + T*p0.velocities[i] + T*p1.velocities[i]) / T**3

        q[i] = a + b*t + c*t**2 + d*t**3
        qdot[i] = b + 2*c*t + 3*d*t**2
        qddot[i] = 2*c + 6*d*t
    return JointTrajectoryPoint(positions=q, velocities=qdot, accelerations=qddot, time_from_start=rospy.Duration(t_abs))

# Returns (q, qdot, qddot) for sampling the JointTrajectory at time t.
# The time t is the time since the trajectory was started.
def sample_traj(traj, t):
    # First point
    if t <= 0.0:
        return copy.deepcopy(traj.points[0])
    # Last point
    if t >= traj.points[-1].time_from_start.to_sec():
        return copy.deepcopy(traj.points[-1])
    
    # Finds the (middle) segment containing t
    i = 0
    while traj.points[i+1].time_from_start.to_sec() < t:
        i += 1
    return cubic_spline_interp(traj.points[i], traj.points[i+1], t)

def close_check(new_start_positions):
    with jpose_lock:
        for i in range(DOF):
            if abs(curr_jpose[i] - new_start_positions[i]) > GAP_TOL_RAD:
                return False

    return True


class xArmTrajAction(object):
    def __init__(self, robot):
        self.server = actionlib.ActionServer("follow_joint_trajectory",FollowJointTrajectoryAction,self.goal_cb, self.cancel_cb, auto_start=False)
        self.goal_handle = None
        self.traj = None
        self.robot = robot
        self.fail_cnt = 0.0
        self.mutex = threading.Lock() # used during traj and goal_handle change
        rospy.loginfo("follow_joint_trajectory server Activated")
        self.update_timer = rospy.Timer(rospy.Duration(1.0/UPDATE_RATE_HZ), self._update)
        # TODO: Need initialize the traj at current single point??

    def start(self):
        self.server.start()


    def goal_cb(self, goal_handle):
        if not self.robot:
            rospy.logerr("Received a goal, but robot is not connected!")
            goal_handle.set_rejected()
            return

        # time.sleep(1.0) # temporary, for safty reation, delay 1.0 sec to execute 
        # make sure self.trajectory name/data is in proper order
        reorder_traj_joints(goal_handle.get_goal().trajectory, JOINT_NAMES)
        
        # check if new trajectory start point is to far-away from current pose
        # Moveit will also help with the check
        if not close_check(goal_handle.get_goal().trajectory.points[0].positions):
            goal_handle.set_rejected()


        # TODO: complete the overriding strategy when there is existing trajectory
        # discard (cancel) the previous goal
        if self.goal_handle:
            self.goal_handle.set_canceled()
            self.goal_handle = None
        rospy.loginfo("Received a new goal!")
        print(goal_handle.get_goal().trajectory.points[-1])

        self.robot.set_state(0)
        with self.mutex:
            now = time.time()
            self.goalStartTime = now # floating-point secs
            self.goal_handle = goal_handle
            self.traj = self.goal_handle.get_goal().trajectory
            self.goal_handle.set_accepted()

            point0 = sample_traj(self.traj, now - self.goalStartTime)
            point0.time_from_start = rospy.Duration(0.0)
            self.traj.points.insert(0, point0)
            


    def cancel_cb(self, goal_handle, immediately=False):
        rospy.loginfo("cancel_cb called!")
        # Only effective when current goal_handle is not None
        if self.goal_handle and self.traj:
            # Option 1: just call Emg_Stop on xArm Hardware
            if immediately:
                self.robot.set_state(4)
            
            # Option 2: Use Cubic interpolation to a graceful stop
            # TODO: add mutex lock
            now = time.time()
            
            stop_start = sample_traj(self.traj, now - self.goalStartTime)
            stop_start.time_from_start = rospy.Duration(0.0)

            stop_end = sample_traj(self.traj, now - self.goalStartTime + STOP_DURATION)
            stop_end.velocities = [0]*DOF
            stop_end.accelerations = [0]*DOF
            stop_end.time_from_start = rospy.Duration(STOP_DURATION)

            with self.mutex:
                if immediately:
                    self.traj.points = [stop_start]
                else:    
                    self.traj.points = [stop_start, stop_end]
                
                self.goal_handle.set_canceled()
                self.goal_handle = None


    def is_finished(self):
        if self.traj == None:
            return True

        jpose_lock.acquire()
        for cur, tar, tol in list(zip(curr_jpose, self.traj.points[-1].positions, P_TOLERANCE)):
            if abs(tar - cur) > tol:
                jpose_lock.release()
                return False

        jpose_lock.release()
        return True


    def _update(self, event):
        if self.robot and self.traj:
            with self.mutex:
                now = time.time()
                if (now - self.goalStartTime) <= self.traj.points[-1].time_from_start.to_sec():
                    self.last_point_sent = False #sending intermediate points
                    # jason comment: calculate current (q, qd, qdd) based on current time point and cmd trajectory series
                    setpoint = sample_traj(self.traj, now - self.goalStartTime)
                    try:
                        # self.robot.send_servoj(999, setpoint.positions, 4 * self.RATE)
                        ret = self.robot.set_servo_angle_j(setpoint.positions, 0, 0, 0)
                        # print(setpoint.positions)
                    except socket.error:
                        pass
                        
                elif not self.last_point_sent:
                    # All intermediate points sent, sending last point to make sure we
                    # reach the goal.
                    # This should solve an issue where the robot does not reach the final
                    # position and errors out due to not reaching the goal point.
                    last_point = self.traj.points[-1]
                    position_in_tol = self.is_finished()
                    # Performing this check to try and catch our error condition.  We will always
                    # send the last point just in case.
                    if not position_in_tol:
                        rospy.logwarn("Trajectory time exceeded and current robot state not at goal, last point required")
                        rospy.logwarn("Current trajectory time: %s, last point time: %s" % \
                                    (now - self.goalStartTime, self.traj.points[-1].time_from_start.to_sec()))

                    setpoint = sample_traj(self.traj, self.traj.points[-1].time_from_start.to_sec())

                    try:
                        self.robot.set_servo_angle_j(setpoint.positions, 0, 0, 0)
                        self.last_point_sent = True
                    except socket.error:
                        pass
                        
                else:  # Off the end
                    if self.goal_handle:
                        position_in_tol = self.is_finished()
                        # TODO: velocity_in_tol = within_tolerance(state.velocity, last_point.velocities, [0.05]*6)
                        if position_in_tol:
                            # The arm reached the goal (and isn't moving).  Succeeding
                            self.goal_handle.set_succeeded()
                            self.goal_handle = None
                            self.fail_cnt = 0.0
                            print(curr_jpose)
                            # rospy.loginfo("Trajectory Execution SUCCESS!")

                        else:
                            self.fail_cnt += 1
                            if(self.fail_cnt/UPDATE_RATE_HZ > EXEC_TIME_TOLERANCE_SEC):
                                self.goal_handle.set_aborted()
                                self.goal_handle = None
                                self.fail_cnt = 0.0
                                rospy.logerr("Execution time out, trajectory aborted!!!")



def update_JS(data):
    global curr_jpose
    # with (threading.Lock) equivalent to acquire()... release()
    with jpose_lock:
        curr_jpose = copy.deepcopy(data.position)

def JSSubscriber():
    rospy.Subscriber("joint_states", JointState, update_JS)
    # if no spin(), program will not respond to "Ctrl+C"
    rospy.spin()


def driver_main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', dest='ip', default="192.168.1.11", help="your robot controler ip")
    parser.add_argument('__name:', help="ROS Node Name")
    parser.add_argument('__log:', help="ROS Node log file Name")
    args = parser.parse_args()

    rospy.init_node('xarm_driver')
    if rospy.get_param("use_sim_time", False):
        rospy.logerr("use_sim_time should NOT be True! Please check your ros parameter!")
        return

    # connect to the real xArm7 hardware with specified IP address 
    xarm = XArmAPI(args.ip, do_not_open=False)
    xarm.motion_enable(True)

    xarm.set_mode(1) # set mode to SERVO MODE
    xarm.set_state(0)

    # start another thread to subscribe to joint state topic
    thread_commander = threading.Thread(name="JSSubscriber", target=JSSubscriber)
    thread_commander.daemon = True # Exit with the main thread
    thread_commander.start()

    server = xArmTrajAction(xarm)
    server.start()

if __name__ == '__main__':

    driver_main()
#!/usr/bin/env python
import argparse
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryActionFeedback, FollowJointTrajectoryActionResult, \
    FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander
import moveit_msgs.msg
import numpy as np


def jog(vx, vy, vz, ang_x, ang_y, ang_z, action_client):
    desired_twist = np.array([vx, vy, vz, ang_x, ang_y, ang_z])

    startingPoint = JointTrajectoryPoint()
    startingPoint.positions = arm_group.get_current_joint_values()
    startingPoint.time_from_start.secs = 0

    trajectory = JointTrajectory()
    trajectory.points = [startingPoint]

    # parameters
    N_points = 300
    Time_total_sec = 5.0
    dt_point2point = Time_total_sec/N_points
    time_from_start = 0.0

    for i in range(1, N_points):
        point = JointTrajectoryPoint()

        jacobian = arm_group.get_jacobian_matrix(trajectory.points[i-1].positions)
        pseudoInverseJacobian = np.linalg.pinv(jacobian)
        jointVelocities = pseudoInverseJacobian.dot(desired_twist)

        point.positions = (trajectory.points[i-1].positions + (jointVelocities*dt_point2point)).tolist()

        time_from_start = time_from_start + dt_point2point
        point.time_from_start = rospy.Duration.from_sec(time_from_start)
        trajectory.points.append(point)

    trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory

    action_client.send_goal(goal)

    action_client.wait_for_result()


def redundancy_resolution(q_i_change, q_index, alpha_scale, action_client):
    desired_twist = np.array([0, 0, 0, 0, 0, 0])

    startingPoint = JointTrajectoryPoint()
    startingPoint.positions = arm_group.get_current_joint_values()
    startingPoint.time_from_start.secs = 0
    q_i_desired = startingPoint.positions[q_index-1] + q_i_change

    trajectory = JointTrajectory()
    trajectory.points = [startingPoint]

    # parameters
    N_points = 300
    Time_total_sec = 5.0
    dt_point2point = Time_total_sec/N_points
    time_from_start = 0.0

    for i in range(1, N_points):
        point = JointTrajectoryPoint()
        # compute null space projection
        jacobian = arm_group.get_jacobian_matrix(trajectory.points[i-1].positions)
        pseudoInverseJacobian = np.linalg.pinv(jacobian) # this mat is 7 by 6
        nullSpaceProjector = \
            np.identity(7) - pseudoInverseJacobian.dot(jacobian) # this mat is 7 by 7, Equation 5.1, I - J+ J
        q_i_current = trajectory.points[i-1].positions[q_index-1]
        gradient_q = np.zeros(7)
        gradient_q[q_index-1] = q_i_current-q_i_desired
        eta = alpha_scale*gradient_q # this vector is 7 by 1, Notes in Asana
        jointVelocities = \
            pseudoInverseJacobian.dot(desired_twist) \
            + nullSpaceProjector.dot(eta)
        # add the desired joint positions to the queue
        point.positions = \
            (trajectory.points[i-1].positions \
            + (jointVelocities*dt_point2point)).tolist()
        time_from_start = time_from_start + dt_point2point
        point.time_from_start = rospy.Duration.from_sec(time_from_start)
        trajectory.points.append(point)

    trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory

    action_client.send_goal(goal)

    action_client.wait_for_result()


if __name__ == '__main__':
    # parsing input arguments
    parser = argparse.ArgumentParser(description='Redundancy Resolution')
    parser.add_argument('-q','--q_angle_change', default=-90.0,
        metavar='[degrees]', type=float,help='desired joint angle change in degrees')
    parser.add_argument('-i','--index', default=1,
        metavar='[ith joint]', type=int,
        help='which joint to set the desired angle (starting from 1st)')
    parser.add_argument('-a','--alpha', default=-0.5,
        metavar='[value]', type=float,help='stepsize scaling parameter alpha')
    args = parser.parse_args()
    q_angle_change_rad = args.q_angle_change/180.0*np.pi
    q_index = args.index
    alpha = -abs(args.alpha)

    moveit_commander.roscpp_initialize('')
    rospy.init_node('robot_jogging')

    client = actionlib.SimpleActionClient('/xarm/xarm7_traj_controller/follow_joint_trajectory',
                                          FollowJointTrajectoryAction)
    client.wait_for_server()

    arm_group_name = "xarm7"
    robot = moveit_commander.RobotCommander("robot_description")
    scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
    arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
    display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                        moveit_msgs.msg.DisplayTrajectory,
                                                        queue_size=20)
    redundancy_resolution(q_angle_change_rad, q_index, alpha, client)


#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg
import tf
import numpy as np

from std_msgs.msg import (Header, String)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from tf.transformations import euler_from_quaternion

import moveit_commander
import moveit_msgs.msg

from moveit_commander import MoveGroupCommander
#from openravepy import *



if __name__ == '__main__':

	#First initialize moveit_commander and rospy.
	print "============ Starting tutorial setup"
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

	#Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
	robot = moveit_commander.RobotCommander()

	#Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
	print "============ Waiting for RVIZ..."
	rospy.sleep(2)
	print "============ Starting tutorial "

	#Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
	scene = moveit_commander.PlanningSceneInterface()

	#Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. In this case the group is the joints in the left arm. This interface can be used to plan and execute motions on the left arm.
	group_both_arms = MoveGroupCommander("both_arms")
	group_both_arms.set_goal_position_tolerance(0.01)
	group_both_arms.set_goal_orientation_tolerance(0.01)

	group_left_arm = MoveGroupCommander("left_arm")
	group_left_arm.set_goal_position_tolerance(0.01)
	group_left_arm.set_goal_orientation_tolerance(0.01)

	group_right_arm = MoveGroupCommander("right_arm")
	group_right_arm.set_goal_position_tolerance(0.01)
	group_right_arm.set_goal_orientation_tolerance(0.01)

	#We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)



	#--Getting Basic Information--#

	#We can get the name of the reference frame for this robot
	#print "============ Reference frame: %s" % group_right_arm.get_planning_frame()

	#We can also print the name of the end-effector link for this group
	#print "============ End-effector link: %s" % group_left_arm.get_end_effector_link()
	
	#We can get a list of all the groups in the robot
	#print "============ Robot Groups:"
	#print robot.get_group_names()

	#Sometimes for debugging it is useful to print the entire state of the robot.
	#print "============ Printing robot state"
	#print robot.get_current_state()
	#print "============"



	# Obtain current pose of left and right end-effector
	left_ee_pose = group_left_arm.get_current_pose()
	right_ee_pose = group_right_arm.get_current_pose()

	P_left_euler = group_left_arm.get_current_rpy()
	#print "Left Euler Angles: ", P_left_euler

	# Initialize separation distance of Baxter's grippers for the hand off
	gripper_separation = 0.1

	# Average pose of left and right end-effectors to determine an initial starting position for the hand-off of the object
	#pose_target_right = geometry_msgs.msg.Pose()
	#pose_target_right.position.x = 0.5*(left_ee_pose.pose.position.x + right_ee_pose.pose.position.x) 
	#pose_target_right.position.y = 0.5*(left_ee_pose.pose.position.y + right_ee_pose.pose.position.y) - gripper_separation
	#pose_target_right.position.z = 0.5*(left_ee_pose.pose.position.z + right_ee_pose.pose.position.z)
	#pose_target_right.orientation.x = 0.56508
	#pose_target_right.orientation.y = -0.5198
	#pose_target_right.orientation.z = -0.54332
	#pose_target_right.orientation.w = -0.33955


	#pose_target_left = geometry_msgs.msg.Pose()
	#pose_target_left.position.x = 0.5*(left_ee_pose.pose.position.x + right_ee_pose.pose.position.x)
	#pose_target_left.position.y = 0.5*(left_ee_pose.pose.position.y + right_ee_pose.pose.position.y) + gripper_separation
	#pose_target_left.position.z = 0.5*(left_ee_pose.pose.position.z + right_ee_pose.pose.position.z)
	#pose_target_left.orientation.x = 0.69283
	#pose_target_left.orientation.y = 0.1977
	#pose_target_left.orientation.z = -0.16912
	#pose_target_left.orientation.w = 0.67253



	# Ensure position of the hand-off is in Baxter's work-space
	#if pose_target_right.position.x < 0.2: 	# x-position is not inside Baxter
	#	pose_target_right.position.x = 0.2
	#	pose_target_left.position.x = 0.2


	#Minimization using euler angle constraints for: JS_to_P(q[0:7],'left')[3,0]-JS_to_P(q[7:14],'right')[3,0]-Handoff_separation[3,0]
	#Success afer 393 iterations - angles do not match up in Moveit however!!
	#Euler angles left: -0.319129504315 1.25166682143 2.82246314848
	#Euler angles right: -0.319129460047 1.52273504464 1.40562709219
	#Q_left = np.array([-0.45187272,0.2454605,1.86214981,0.73705328,-0.02303775,2.094,-1.48406402])
	#Q_right = np.array([-0.47271541,0.30031665,2.07864775,1.00307422,-0.34570451,2.094,-1.28628682])

	#Minimization using euler angle constraints for: JS_to_PrPlRrl(q)[6,0] - Handoff_separation[3,0]
	#Failed after 1852 iterations --> 'Inequality constraints incompatible'
	#Euler angles left: 2.05432296683 0.974265057754 0.877991242072
	#Euler angles right: -0.743941934809 -0.578199288289 2.18747774161
	#Configuration apparently not within joint limit tolerances
	#Q_left = np.array([ 1.84517617, -0.72247078, -0.41088249,  0.95745551,  0.82491167,  2.62908057, -2.07438494])
	#Q_right = np.array([ -1.70576178e+00,   6.07833180e-01,   5.86496435e-01,   7.66755357e-03,  -9.83686779e-01,  -1.35853486e+00,  -8.21096952e+00])


	#Minimization using distance constraints for: JS_to_PrPlRrl(q)[0,0] - JS_to_PrPlRrl(q)[3,0] - Handoff_separation[0,0]
	#Success after 139 iterations ~ 5 minutes
	#Collision occured between forearms, possibly from orientation. Positions matched up!
	#Q_left = np.array([-0.55300538,-0.03706502,-0.86447796,0.82440213,-0.38401233,-1.3826278,1.01142092])
	#Q_right = np.array([1.12487157,0.0916066,0.57593316,0.18093991,-1.63044579,-0.45322471,0.98991324])

	#Added constraint R_right[0,0] = R_left[0,1]
	#Collision between head link 1 and right upper forearm
	#result = np.array([-0.09176617, -0.38031457, -0.23897933,  0.95991322,  1.0213436,  -0.70647256, -0.62593032, 1.25315321, -0.87147597,  1.77136804,  1.13627579,  0.64865524,  1.28456289,  0.47474063])
	#Added constraint R_right[2,1] = R_left[2,0]
	#Possibly proper orientation, but in vertical direction
	#result = np.array([ 0.21005165,  0.11059057, -1.72637393,  0.35251872, -1.41167145, -1.53800182,  0.619337, 1.17354539,  0.06471717, -1.01183087,  1.92790736, -1.28521065,  2.094,  0.08916447])
	#Added constraint R_right[1,2] = R_left[1,2]
	#Possibly proper orientation, but in x-direction
	#result = np.array([ 0.3985867,   0.07086207, -2.15967208,  0.25667434,  0.63271133,  0.97211777,  1.4844826, 0.11074489,  0.69245848,  3.05417994,  1.65912968,  0.73117134,  2.094,  1.90841508])
	#Added constraint R_right[2,0] = R_left[2,1]
	#Appears to be correct orientation, 132 iterations
	#result = np.array([ 0.09443072, -0.52521359, -1.16577959 , 2.17922088,  2.04029513, -0.74734333, -0.58585548, -0.29722334, -0.76102714,  0.9978483,   0.75572041,  1.00379169,  1.99751188 , 0.25848372])

	#Full constraints: R_right[2,0] = R_left[2,1]
	#Handoff occuring in vertical direction, and therefore still 0.1m apart in x-direction
	#result = np.array([-0.70252588,  0.2506697,  -0.59921154, -0.05,  -2.5750736,   2.094,  3.059, 1.39819159, -0.18598255,  3.05417994,  0.21150206,  3.059,   1.70179822, -1.02305348])
	#Full constraints: R_right[2,1] = R_left[2,0]
	#Fail after 525 iterations - Inequality constraints incompatible
	#result = np.array([ 0.6639237,  -1.0516018,  -0.73870185,  2.31373659, -0.20999322,  1.9208306, -1.99140418, 1.22144366,  1.01706032,  1.62747978,  2.57366797,  2.37392543,  1.64364215,  0.74510502])
	#Full constraints: R_right[1,2] = R_left[1,2]
	#Let run for 30 minutes, no solution found, killed process
	#Full constraints: R_right[0,0],[R_left[0,1]
	#Handoff occuring in proper direction, but 70 deg rotated, and position constraints don't hold anymore
	#result = np.array([ 1.70167994, -0.57741879, -2.32989003,  2.618, 2.92190473, -0.10883174,  3.059,0.93421685,  0.46039246,  2.30975355,  1.29052514,  2.69636996, -1.57079633, -1.66197632])
	#Full constraints: R_right[0,0],R_left[0,1],R_right[1,0],R_left[1,1]
	#Compared to previous, angle away from expected was less (25-30 deg), but took more iterations (171 vs 70)
	result = np.array([ 0.89478734, -1.17897522, -1.34234682,  1.38186236,  1.95370677, -1.56835601,  3.05284001, 1.13116115, -0.03660525,  2.79703457,  0.89832103,  1.82973614, -1.33333512, -0.67531548])



	#No added constraint
	#result = np.array([0.10657421,  0.17023356, -1.56176755,  0.25789247,  0.25192917, -0.66405269,  1.29805017, 0.18426757,  0.43598581,  2.71962098,  0.56813731,  1.96014855,  1.48764975,  3.05213457])

	joints = {'left_s0': result[0], 'left_s1': result[1], 'left_e0': result[2], 'left_e1': result[3], 'left_w0': result[4], 'left_w1': result[5], 'left_w2': result[6], 'right_s0': result[7], 'right_s1': result[8], 'right_e0': result[9], 'right_e1': result[10], 'right_w0': result[11], 'right_w1': result[12], 'right_w2': result[13]}
	#joints = {'left_s0': Q_left[0], 'left_s1': Q_left[1], 'left_e0': Q_left[2], 'left_e1': Q_left[3], 'left_w0': Q_left[4], 'left_w1': Q_left[5], 'left_w2': Q_left[6], 'right_s0': Q_right[0], 'right_s1': Q_right[1], 'right_e0': Q_right[2], 'right_e1': Q_right[3], 'right_w0': Q_right[4], 'right_w1': Q_right[5], 'right_w2': Q_right[6]}

	group_both_arms.set_joint_value_target(joints)
	plan_both = group_both_arms.plan()



	# Generate plans for both_arms group and show if successful
	#print "============ Generating plan for both arms with poses"
	#print pose_target_right.position
	#print pose_target_left.position
	#group_both_arms.set_pose_target(pose_target_right, 'right_gripper')
	#group_both_arms.set_pose_target(pose_target_left, 'left_gripper')

	#plan_both = group_both_arms.plan()


	# If orientation might be the problem, alter both by a small amount and try planning again
	#if plan_both == False:

	#	quaternion_right = (pose_target_right.orientation.x,pose_target_right.orientation.y,pose_target_right.orientation.z,pose_target_right.orientation.w)
	#	(roll_r,pitch_r,yaw_r) = euler_from_quaternion(quaternion_right)

	#	quaternion_left = (pose_target_left.orientation.x,pose_target_left.orientation.y,pose_target_left.orientation.z,pose_target_left.orientation.w)
	#	(roll_l,pitch_l,yaw_l) = euler_from_quaternion(quaternion_left)


	#print plan_both

#!/usr/bin/env python

import numpy as np
import rospy
import math
import tf
import sys
import copy
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg

from scipy.optimize import minimize
from math import sqrt
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from tf.transformations import euler_from_matrix
from std_msgs.msg import (Header, String)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from moveit_commander import MoveGroupCommander




# Initialize moveit commander and move group commander
def InitializeMoveitCommander():

	#First initialize moveit_commander 
	print "============ Starting tutorial setup"
	moveit_commander.roscpp_initialize(sys.argv)

	#Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
	robot = moveit_commander.RobotCommander()

	#Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
	print "============ Waiting for RVIZ..."
	rospy.sleep(1)
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


	# Obtain current poses of left and right end-effectors: [x,y,z,roll,pitch,yaw].T
	P_left_pose = group_left_arm.get_current_pose()
	P_right_pose = group_right_arm.get_current_pose()
	P_left_euler = group_left_arm.get_current_rpy()
	P_right_euler = group_right_arm.get_current_rpy()

	global P_left_current, P_right_current
	P_left_current = np.array([[P_left_pose.pose.position.x],[P_left_pose.pose.position.y],[P_left_pose.pose.position.z],[P_left_euler[0]],[P_left_euler[1]],[P_left_euler[2]]])
	P_right_current = np.array([[P_right_pose.pose.position.x],[P_right_pose.pose.position.y],[P_right_pose.pose.position.z],[P_right_euler[0]],[P_right_euler[1]],[P_right_euler[1]]])
	print "\nCurrent pose of left and right EE: \n", np.transpose(P_left_current), "\n", np.transpose(P_right_current)




# Takes configuration variables in joint space and returns the end-effector position and Euler XYZ angles
def JS_to_P(q,arm):

	robot = URDF.from_xml_file("/home/josh/catkin_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")

	kdl_kin = KDLKinematics(robot, "base", str(arm)+"_gripper")

	q_correct_order = np.array([q[0], q[1], q[2], q[3], q[4], q[5], q[6]])

	T = kdl_kin.forward(q_correct_order)
	R = T[:3,:3]

	x = T[0,3]
	y = T[1,3]
	z = T[2,3]

	roll,pitch,yaw = euler_from_matrix(R, 'sxyz')

	P = np.array([[x],[y],[z],[roll],[pitch],[yaw]])

	return P



# Takes configuration variables in joint space and returns the end-effector position and Euler XYZ angles
def JS_to_PrPlRrl(q):

	robot = URDF.from_xml_file("/home/josh/catkin_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")

	kdl_kin_left = KDLKinematics(robot, "base", "left_gripper")
	kdl_kin_right = KDLKinematics(robot, "base", "right_gripper")


	T_left = kdl_kin_left.forward(q[0:7])
	R_left = np.array(T_left[:3,:3])

	T_right = kdl_kin_right.forward(q[7:14])
	R_right = np.array(T_right[:3,:3])


	x_left = T_left[0,3]
	y_left = T_left[1,3]
	z_left = T_left[2,3]

	x_right = T_right[0,3]
	y_right = T_right[1,3]
	z_right = T_right[2,3]


	R_rl = np.dot(np.transpose(R_right), R_left)

	roll,pitch,yaw = euler_from_matrix(R_rl, 'sxyz')

	P = np.array([[x_left],[y_left],[z_left],[x_right],[y_right],[z_right],[roll],[pitch],[yaw],[R_right[0,0]],[R_left[0,1]],[R_right[1,0]],[R_left[1,1]]])

	return P




# Normalize P1-P2: square root of sum of squares
def norm(P1, P2):

	normalized_result = sqrt( (P1[0,0]-P2[0,0])**2 + (P1[1,0]-P2[1,0])**2 + (P1[2,0]-P2[2,0])**2 + (P1[3,0]-P2[3,0])**2 + (P1[4,0]-P2[4,0])**2 + (P1[5,0]-P2[5,0])**2 )

	return normalized_result



# Minimization function to find a hand-off pose starting with initial right and left end-effector poses
minimization = lambda q: (norm(JS_to_P(q[0:7],'left'), P_left_current) + norm(JS_to_P(q[7:14],'right'), P_right_current))



# Main portion of code
def main():

	# Initialize node
	rospy.init_node('minimization')

	# Start Moveit Commander
	InitializeMoveitCommander()


	# Initial guess
	x0 = np.full((14,1), 0.5)

	# Bounds for SLSQP: s0, s1, e0, e1, w0, w1, w2 (left,right)
	bnds = ( (-1.70167993878, 1.70167993878), (-2.147, 1.047), (-3.05417993878, 3.05417993878), (-0.05, 2.618), (-3.059, 3.059), (-1.57079632679, 2.094), (-3.059, 3.059),(-1.70167993878, 1.70167993878), (-2.147, 1.047), (-3.05417993878, 3.05417993878), (-0.05, 2.618),  (-3.059, 3.059), (-1.57079632679, 2.094), (-3.059, 3.059)) # lower and upper bounds for each q (length 14)

	# Constraint equality
	Handoff_separation = np.array([[0.0],[0.1],[0.0],[math.pi/1.],[0],[-math.pi/2.]])
	cons = ({'type': 'eq', 'fun': lambda q: JS_to_P(q[0:7],'left')[3,0]-JS_to_P(q[7:14],'right')[3,0]-Handoff_separation[3,0]},
			{'type': 'eq', 'fun': lambda q: JS_to_P(q[0:7],'left')[4,0]-JS_to_P(q[7:14],'right')[3,0]-Handoff_separation[4,0]}, 
			{'type': 'eq', 'fun': lambda q: JS_to_P(q[0:7],'left')[5,0]-JS_to_P(q[7:14],'right')[3,0]-Handoff_separation[5,0]})
	
	cons2 = ({'type': 'eq', 'fun': lambda q: JS_to_P(q[0:7],'left')[3,0]-JS_to_P(q[7:14],'right')[3,0]-Handoff_separation[3,0]},
			 {'type': 'eq', 'fun': lambda q: JS_to_P(q[0:7],'left')[4,0]-JS_to_P(q[7:14],'right')[3,0]-Handoff_separation[4,0]}, 
			 {'type': 'eq', 'fun': lambda q: JS_to_P(q[0:7],'left')[5,0]-JS_to_P(q[7:14],'right')[3,0]-Handoff_separation[5,0]},
			 {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[6,0] - Handoff_separation[3,0]},
			 {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[7,0] - Handoff_separation[4,0]},
			 {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[8,0] - Handoff_separation[5,0]})

	cons3 = ({'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[0,0] - JS_to_PrPlRrl(q)[3,0] - Handoff_separation[0,0]},
			 {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[1,0] - JS_to_PrPlRrl(q)[4,0] - Handoff_separation[1,0]},
			 {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[2,0] - JS_to_PrPlRrl(q)[5,0] - Handoff_separation[2,0]})

	cons4 = ({'type': 'eq', 'fun': lambda q: math.fabs(JS_to_PrPlRrl(q)[6,0]) - Handoff_separation[3,0]},
			 {'type': 'eq', 'fun': lambda q: math.fabs(JS_to_PrPlRrl(q)[7,0]) - Handoff_separation[4,0]},
			 {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[8,0] - Handoff_separation[5,0]},
			 {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[9,0] - JS_to_PrPlRrl(q)[10,0]})

	cons5 = ({'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[0,0] - JS_to_PrPlRrl(q)[3,0] - Handoff_separation[0,0]}, 	#x-distance = 0.1*sin(acos(R_right[0,0])) //JS_to_PrPlRrl(q)[9,0]
			 {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[1,0] - JS_to_PrPlRrl(q)[4,0] - Handoff_separation[1,0]}, 	#y-distance = 0.1*cos(acos(R_right[0,0]))
			 {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[2,0] - JS_to_PrPlRrl(q)[5,0] - Handoff_separation[2,0]}, 	#z-distance = 0
			 {'type': 'eq', 'fun': lambda q: math.fabs(JS_to_PrPlRrl(q)[6,0]) - Handoff_separation[3,0]},			   	#roll
			 {'type': 'eq', 'fun': lambda q: math.fabs(JS_to_PrPlRrl(q)[7,0]) - Handoff_separation[4,0]},				#pitch
			 {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[8,0] - Handoff_separation[5,0]},							#yaw
			 {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[9,0] - JS_to_PrPlRrl(q)[10,0]},							#EE's pointed towards each other instead of away
			 {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[11,0] - JS_to_PrPlRrl(q)[12,0]})							#EE's pointed towards each other instead of away



	# Minimization
	result = minimize(minimization, x0, method='SLSQP', bounds=bnds, constraints=cons5, tol=None, options={'maxiter': 10000})



	print "\nNumber of iterations: \n", result.success, result.nit
	print result


	robot = URDF.from_xml_file("/home/josh/catkin_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")

	kdl_kin_left = KDLKinematics(robot, "base", "left_gripper")
	kdl_kin_right = KDLKinematics(robot, "base", "right_gripper")

	q_left = result.x[0:7]
	q_right = result.x[7:14]

	pose_left = kdl_kin_left.forward(q_left)
	pose_right = kdl_kin_right.forward(q_right)

	print "\nQ left: \n",q_left
	print "Q right: \n",q_right
	print "\nPose left: \n",pose_left
	print "Pose right: \n",pose_right

	R_left = pose_left[:3,:3]
	R_right = pose_right[:3,:3]

	X_left,Y_left,Z_left = euler_from_matrix(R_left, 'sxyz')
	X_right,Y_right,Z_right = euler_from_matrix(R_right, 'sxyz')

	q = np.array([q_left[0],q_left[1],q_left[2],q_left[3],q_left[4],q_left[5],q_left	[6],q_right[0],q_right[1],q_right[2],q_right[3],q_right[4],q_right[5],q_right[6]])
	P = JS_to_PrPlRrl(q)
	print "\nConstraint Equation Results: \n",P

	print "\nR_left: \n",R_left
	print "\nR_right: \n",R_right

	print "\nEuler angles left: \n", X_left,Y_left,Z_left
	print "Euler angles right: \n", X_right,Y_right,Z_right


	#s0, s1, e0, e1, w0, w1, w2 
	#joints = {'left_s0': result[0], 'left_s1': result[1], 'left_e0': result[2], 'left_e1': result[3], 'left_w0': result[4], 'left_w1': result[5], 'left_w2': result[6], 'right_s0': result[7], 'right_s1': result[8], 'right_e0': result[9], 'right_e1': result[10], 'right_w0': result[11], 'right_w1': result[12], 'right_w2': result[13]}

	#group_both_arms.set_joint_value_target(joints)
	#plan_both = group_both_arms.plan()





if __name__ == '__main__':
	main()

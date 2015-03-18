#!/usr/bin/env python

import numpy as np
import rospy
import math
import tf

from scipy.optimize import minimize
from math import sqrt
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from tf.transformations import euler_from_matrix, quaternion_from_matrix




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
def JS_to_quat(q,arm):

	robot = URDF.from_xml_file("/home/josh/catkin_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")

	kdl_kin = KDLKinematics(robot, "base", str(arm)+"_gripper")

	q_correct_order = np.array([q[0], q[1], q[2], q[3], q[4], q[5], q[6]])

	T = kdl_kin.forward(q_correct_order)
	R = T[:3,:3]

	q = quaternion_from_matrix(T)

	x = T[0,3]
	y = T[1,3]
	z = T[2,3]

	P = np.array([[x],[y],[z],[q[0]],[q[1]],[q[2]],[q[3]]])

	return P



# Normalize P1-P2: square root of sum of squares
def norm(P1, P2):

	normalized_result = sqrt( (P1[0,0]-P2[0,0])**2.0 + (P1[1,0]-P2[1,0])**2.0 + (P1[2,0]-P2[2,0])**2.0 + (P1[3,0]-P2[3,0])**2.0 + (P1[4,0]-P2[4,0])**2.0 + (P1[5,0]-P2[5,0])**2.0 )

	return normalized_result



# Minimization function to find a pose starting from x0 to P_right_goal
minimization = lambda q: norm( JS_to_P(q[0:7],'right'), P_right_goal )



# Main portion of code
def main():
	# Initialize node
	#rospy.init_node('Minimization')

	# Define goal position for right end-effector
	global P_right_goal
	P_right_goal = np.array([[0.8],[-0.3],[0.0],[0.7580154849109062],[-1.563242895215259],[2.3810247415697874]])
	P_right_goal_quat = np.array([[0.8],[-0.3],[0.0],[0.705164159846],[0.000936004730899],[0.709038164992],[0.00274082988659]])

	# Initial guess at configuration variables q_right
	x0 = np.full((7,1), 0.5)
	#x0 = np.array([-0.11,0.40,0.95,1.51,-0.90,-1.11,2.38])

	# Bounds for SLSQP: s0, s1, e0, e1, w0, w1, w2 (right)
	bnds = ((-1.70167993878, 1.70167993878), (-2.147, 1.047), (-3.05417993878, 3.05417993878), (-0.05, 2.618), (-3.059, 3.059), (-1.57079632679, 2.094), (-3.059, 3.059)) # lower and upper bounds for each q (length 7)

	# Constraint equality
	cons = ({'type': 'eq', 'fun': lambda q: JS_to_P(q[0:7],'right')[3,0]-P_right_goal[3,0]},
			{'type': 'eq', 'fun': lambda q: JS_to_P(q[0:7],'right')[4,0]-P_right_goal[4,0]}, 
			{'type': 'eq', 'fun': lambda q: JS_to_P(q[0:7],'right')[5,0]-P_right_goal[5,0]})

	cons1 =({'type': 'eq', 'fun': lambda q: JS_to_quat(q[0:7],'right')[3,0]-P_right_goal_quat[3,0]},
			{'type': 'eq', 'fun': lambda q: JS_to_quat(q[0:7],'right')[4,0]-P_right_goal_quat[4,0]}, 
			{'type': 'eq', 'fun': lambda q: JS_to_quat(q[0:7],'right')[5,0]-P_right_goal_quat[5,0]},
			{'type': 'eq', 'fun': lambda q: JS_to_quat(q[0:7],'right')[6,0]-P_right_goal_quat[6,0]})
			#{'type': 'eq', 'fun': lambda q: JS_to_quat(q[0:7],'right')[0,0]-P_right_goal_quat[0,0]},
			#{'type': 'eq', 'fun': lambda q: JS_to_quat(q[0:7],'right')[1,0]-P_right_goal_quat[1,0]},
			#{'type': 'eq', 'fun': lambda q: JS_to_quat(q[0:7],'right')[2,0]-P_right_goal_quat[2,0]})


	# Minimization
	result = minimize(minimization, x0, method='SLSQP', bounds=bnds, constraints=cons, tol=None, options={'maxiter': 1000})

	print "\n Number of iterations: \n", result.success, result.nit
	#print "\n IK answer: \n", np.array([0.9236827612602686,1.4712713057421967,-0.1042813528914594,0.3802553766204043,-0.8860336003372191,-1.0064829097388408,2.34976894473572])



	robot = URDF.from_xml_file("/home/josh/catkin_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")
	kdl_kin_right = KDLKinematics(robot, "base", "right_gripper")

	q_right = result.x

	pose_right = kdl_kin_right.forward(q_right)

	print "\n Q right: \n",q_right
	print "\n Pose right: \n",pose_right

	R = pose_right[:3,:3]
	X,Y,Z = euler_from_matrix(R, 'sxyz')

	quat = quaternion_from_matrix(pose_right)

	print "\n Euler angles: \n", X,Y,Z
	print "\n Quaternion: \n",quat

	print "\n\n\n Result: \n", result




if __name__ == '__main__':
	main()

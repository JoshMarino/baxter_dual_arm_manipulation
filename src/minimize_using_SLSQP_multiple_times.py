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

	roll,pitch,yaw = euler_from_matrix(R, 'rxyz')

	P = np.array([[x],[y],[z],[roll],[pitch],[yaw]])

	return P



# Normalize P1-P2: square root of sum of squares
def norm(P1, P2):

	normalized_result = sqrt( (P1[0,0]-P2[0,0])**2 + (P1[1,0]-P2[1,0])**2 + (P1[2,0]-P2[2,0])**2 ) #+ (P1[3,0]-P2[3,0])**2 + (P1[4,0]-P2[4,0])**2 + (P1[5,0]-P2[5,0])**2 )

	return normalized_result



# Minimization function to find a hand-off pose starting with initial right and left end-effector poses
minimization = lambda q: (norm(JS_to_P(q[0:7],'left'), P_left_current) + norm(JS_to_P(q[7:14],'right'), P_right_current))




# Returns set of joint angles from SLSQP minimization
def JointAnglesHandOffPose(P_left_current, P_right_current):


	# Initial guess
	x0 = np.full((14,1), 0.5)

	# Bounds for SLSQP: s0, s1, e0, e1, w0, w1, w2 (left,right)
	bnds = ( (-1.70167993878, 1.70167993878), (-2.147, 1.047), (-3.05417993878, 3.05417993878), (-0.05, 2.618), (-3.059, 3.059), (-1.57079632679, 2.094), (-3.059, 3.059),(-1.70167993878, 1.70167993878), (-2.147, 1.047), (-3.05417993878, 3.05417993878), (-0.05, 2.618),  (-3.059, 3.059), (-1.57079632679, 2.094), (-3.059, 3.059)) # lower and upper bounds for each q (length 14)

	# Constraint equality
	Handoff_separation = np.array([[0.0],[0.1],[0.0],[0.0],[math.pi/2.],[math.pi]])
	cons = ({'type': 'eq', 'fun': lambda q: JS_to_P(q[0:7],'left') - JS_to_P(q[7:14],'right') - Handoff_separation})

	# Minimization
	JointAngleResult = minimize(minimization, x0, method='SLSQP', bounds=bnds, constraints=cons, tol=0.005, options={'maxiter': 500})

	print "Results of SLSQP Minimization: ", result

	return JointAngleResult





# Main portion of code
def main():

	# Initialize node
    rospy.init_node('Minimization')

	# Start Moveit Commander
	InitializeMoveitCommander()


	# Minimize hand-off pose in joint angle space - performed 10 times to obtain different results
	JointAngleResult = np.full((40,3), None) # [joint_angles,trajectory_plan,trajectory_time]

	for i in range (0,10):
		JointAngleResult[4*i,0] = JointAnglesHandOffPose(P_left_current, P_right_current)

		#s0, s1, e0, e1, w0, w1, w2 
		joints_left = {'left_s0': JointAngleResult[i,0][0], 'left_s1': JointAngleResult[i,0][1], 'left_e0': JointAngleResult[i,0][2], 'left_e1': JointAngleResult[i,0][3], 'left_w0': JointAngleResult[i,0][4], 'left_w1': JointAngleResult[i,0][5], 'left_w2': JointAngleResult[i,0][6]}
		joints_right = {'right_s0': JointAngleResult[i,0][7], 'right_s1': JointAngleResult[i,0][8], 'right_e0': JointAngleResult[i,0][9], 'right_e1': JointAngleResult[i,0][10], 'right_w0': JointAngleResult[i,0][11], 'right_w1': JointAngleResult[i,0][12], 'right_w2': JointAngleResult[i,0][13]}

		group_both_arms.set_joint_value_target(joints_left, 'left_gripper')
		group_both_arms.set_joint_value_target(joints_right, 'right_gripper')

		plan_both = group_both_arms.plan()

		JointAngleResult[4*i,1] = plan_both
		JointAngleResult[4*i,2] = plan_both.joint_trajectory.points[len(plan_both.joint_trajectory.points)-1].time_from_start


		# Alter hand-off position (not pose) by a random amount 4 times in order to see if MoveIt returns a lesser trajectory time
		Pose_ee_left = JS_to_P(JointAngleResult[0:7,0],'left')
		Pose_ee_right = JS_to_P(JointAngleResult[7:14,0],'right')

		for j in range (0,4):

			JointAngleResult[4*i+j,0] = None

			random1 = random.uniform(-0.1,0.1) # +/- 4 inches
			random2 = random.uniform(-0.1,0.1)
			random3 = random.uniform(-0.1,0.1)

			pose_left = [Pose_ee_left[0,0]+random1,Pose_ee_left[1,0]+random2,Pose_ee_left[2,0]+random3,Pose_ee_left[3,0],Pose_ee_left[4,0],Pose_ee_left[5,0]]
			pose_right = [Pose_ee_right[0,0]-random1,Pose_ee_right[1,0]-random2,Pose_ee_right[2,0]-random3,Pose_ee_right[3,0],Pose_ee_right[4,0],Pose_ee_right[5,0]]

			group_both_arms.set_pose_target(pose_left, 'left_gripper')
			group_both_arms.set_pose_target(pose_right, 'right_gripper')

			plan_both = group_both_arms.plan()

			JointAngleResult[4*i+j,1] = plan_both
			JointAngleResult[4*i+j,2] = plan_both.joint_trajectory.points[len(plan_both.joint_trajectory.points)-1].time_from_start


	# Find trajectory that took the least amount of time and execute it
	minimum_time_entry =  np.nanargmin(JointAngleResult[:,2])

	group_both_arms.execute(JointAngleResult[minimum_time_entry,1])



if __name__ == '__main__':
	main()

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
import random

from scipy.optimize import minimize
from math import sqrt
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from tf.transformations import euler_from_matrix
from std_msgs.msg import (Header, String)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from moveit_commander import MoveGroupCommander


first_flag = False


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
	global group_both_arms, group_left_arm, group_right_arm
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

	if arm == 'right':
		T = kdl_kin_right.forward(q)
	elif arm == 'left':
		T = kdl_kin_left.forward(q)

	R = T[:3,:3]

	x = T[0,3]
	y = T[1,3]
	z = T[2,3]

	roll,pitch,yaw = euler_from_matrix(R, 'sxyz')

	P = np.array([[x],[y],[z],[roll],[pitch],[yaw]])

	return P



# Takes configuration variables in joint space and returns the end-effector position and Euler XYZ angles
def JS_to_PrPlRrl(q):

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


	T_rl = np.dot(InverseTransformationMatrix(T_right), T_left)
	sep_dist_x = T_rl[0,3]
	sep_dist_y = T_rl[1,3]
	sep_dist_z = T_rl[2,3]

	R_rl = np.dot(np.transpose(R_right), R_left)

	roll,pitch,yaw = euler_from_matrix(R_rl, 'sxyz')

	P = np.array([[x_left],[y_left],[z_left],[x_right],[y_right],[z_right],[roll],[pitch],[yaw],[sep_dist_x],[sep_dist_y],[sep_dist_z]])

	return P



# Calculates the inverse of a transformation maxtrix T
def InverseTransformationMatrix(T):

	T_inv = np.full( (4,4), 0)

	T_inv[:3,:3] = np.transpose(T[:3,:3])

	T_inv[:3,3:4] = -np.dot( np.transpose(T[:3,:3]), np.array([[T[0,3]], [T[1,3]], [T[2,3]]]) )

	T_inv[3,3] = 1 

	return T_inv



# Normalize P1-P2: square root of sum of squares
def norm(P1, P2):

	normalized_result = sqrt( (P1[0,0]-P2[0,0])**2 + (P1[1,0]-P2[1,0])**2 + (P1[2,0]-P2[2,0])**2 + (P1[3,0]-P2[3,0])**2 + (P1[4,0]-P2[4,0])**2 + (P1[5,0]-P2[5,0])**2 )

	return normalized_result



# Minimization function to find a hand-off pose starting with initial right and left end-effector poses
minimization = lambda q: (norm(JS_to_P(q[0:7],'left'), P_left_current) + norm(JS_to_P(q[7:14],'right'), P_right_current))



# Estimating jacobian (gradient) of minimization function
def jacobian(q):

	jac_left = kdl_kin_left.jacobian(q[0:7])
	jacinv_left = np.linalg.pinv(jac_left)

	jac_right = kdl_kin_right.jacobian(q[7:14])
	jacinv_right = np.linalg.pinv(jac_right)

	jacobian_left = np.array([ (jacinv_left[i,0] + jacinv_left[i,1] + jacinv_left[i,2] + jacinv_left[i,3] + jacinv_left[i,4] + jacinv_left[i,5]) for i in range(7) ])
	jacobian_right = np.array([ (jacinv_right[i,0] + jacinv_right[i,1] + jacinv_right[i,2] + jacinv_right[i,3] + jacinv_right[i,4] + jacinv_right[i,5]) for i in range(7) ])

	jacobian = np.array([jacobian_left[0],jacobian_left[1],jacobian_left[2],jacobian_left[3],jacobian_left[4],jacobian_left[5],jacobian_left[6],jacobian_right[0],jacobian_right[1],jacobian_right[2],jacobian_right[3],jacobian_right[4],jacobian_right[5],jacobian_right[6]])

	return jacobian



# Returns set of joint angles from SLSQP minimization
def JointAnglesHandOffPose():

	# Initial guess
	initial_left = Pose()
	initial_left.position=Point(
		            x= 0.3,#(P_left_current[0,0] + P_right_current[0,0])/2.,
		            y= (P_left_current[1,0] + P_right_current[1,0])/2.,
		            z= (P_left_current[2,0] + P_right_current[2,0])/2.,
		        )
	initial_left.orientation=Quaternion(
		            x=0.0,
		            y=0.0,
		            z=0.0,
		            w=1.0,
		        )

	initial_right = Pose()
	initial_right.position=Point(
		            x= 0.3,#(P_left_current[0,0] + P_right_current[0,0])/2.,
		            y= (P_left_current[1,0] + P_right_current[1,0])/2.,
		            z= (P_left_current[2,0] + P_right_current[2,0])/2.,
		        )
	initial_right.orientation=Quaternion(
		            x=0.0,
		            y=0.0,
		            z=0.0,
		            w=1.0,
		        )

	x0_left = kdl_kin_left.inverse(initial_left)
	x0_right = kdl_kin_left.inverse(initial_right)

	x0 = np.array([[x0_left[0]],[x0_left[1]],[x0_left[2]],[x0_left[3]],[x0_left[4]],[x0_left[5]],[x0_left[6]],[x0_right[0]],[x0_right[1]],[x0_right[2]],[x0_right[3]],[x0_right[4]],[x0_right[5]],[x0_right[6]]])

	x0 = x0 + random.uniform(-0.075,0.075)


	# Bounds for SLSQP: s0, s1, e0, e1, w0, w1, w2 (left,right)
	bnds = ( (-1.70167993878, 1.70167993878), (-2.147, 1.047), (-3.05417993878, 3.05417993878), (-0.05, 2.618), (-3.059, 3.059), (-1.57079632679, 2.094), (-3.059, 3.059),(-1.70167993878, 1.70167993878), (-2.147, 1.047), (-3.05417993878, 3.05417993878), (-0.05, 2.618),  (-3.059, 3.059), (-1.57079632679, 2.094), (-3.059, 3.059)) # lower and upper bounds for each q (length 14)

	# Constraint equality
	Handoff_separation = np.array([[0.0],[0.1],[0.0],[math.pi/1.],[0],[-math.pi/2.]])
	cons = ({'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[9,0]  - Handoff_separation[0,0]}, 				#x-sep-distance = 0
			{'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[10,0]  - Handoff_separation[2,0]}, 			#y-sep-distance = 0
			{'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[11,0]  - Handoff_separation[1,0]}, 			#z-sep-distance = 0.1
			{'type': 'eq', 'fun': lambda q: math.fabs(JS_to_PrPlRrl(q)[6,0]) - Handoff_separation[3,0]},   	#roll = pi
			{'type': 'eq', 'fun': lambda q: math.fabs(JS_to_PrPlRrl(q)[7,0]) - Handoff_separation[4,0]},	#pitch = 0
			{'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[8,0] - Handoff_separation[5,0]},				#yaw = -pi/2
			#{'type': 'ineq', 'fun': lambda q: JS_to_PrPlRrl(q)[0,0]-0.3},									#x-pos > 0.3 m to help avoid collisions
			{'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[0,0] - (P_left_current[0,0] + P_right_current[0,0])/2.},
			{'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[1,0] - (P_left_current[1,0] + P_right_current[1,0])/2.})
			#{'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[0,0] - (P_left_current[0,0] + P_right_current[0,0])/2.})


	# Minimization
	result = minimize(minimization, x0, method='SLSQP', jac=jacobian, bounds=bnds, constraints=cons, tol=0.1, options={'maxiter': 30})

	print "\nNumber of iterations: \n", result.success, result.nit
	print result

	if result.success == False:
		q = np.array([99,99,99,99,99,99,99,99,99,99,99,99,99,99])
		print result.message
	elif result.success == True:
		q_left = result.x[0:7]
		q_right = result.x[7:14]
		q = np.array([q_left[0],q_left[1],q_left[2],q_left[3],q_left[4],q_left[5],q_left[6],q_right[0],q_right[1],q_right[2],q_right[3],q_right[4],q_right[5],q_right[6]])

	return q



# Listens for a button press to tell main to run the minimization process
def checkButtonPress(msg):

    pose = msg.pose

    position_new = pose.position
    orientation_new = pose.orientation

    global first_flag

    pose_stocking[0,0] = position_new.x
    pose_stocking[1,0] = position_new.y
    pose_stocking[2,0] = position_new.z

    first_flag = True

    return



# Main portion of code
def main():

	# Initialize node
	rospy.init_node('minimization')

	# Start Moveit Commander
	InitializeMoveitCommander()

	# Create a subscriber to listen for a button press on Baxter which means to run the minimization process once.
    rospy.Subscriber("/pose/stocking",PoseStamped,checkButtonPress)


	# Initialization of KDL Kinematics for right and left grippers
	robot = URDF.from_xml_file("/home/josh/catkin_ws/src/baxter_common/baxter_description/urdf/baxter.urdf")

	global kdl_kin_left, kdl_kin_right
	kdl_kin_left = KDLKinematics(robot, "base", "left_gripper")
	kdl_kin_right = KDLKinematics(robot, "base", "right_gripper")


	# Continuously be listening for the flag to indicate to run the minimization process once
    while not rospy.is_shutdown():

		if first_flag == True:

			# Minimize hand-off pose - minimization performed 5 times - MoveIt! planned 10 times each - to obtain different results
			Trajectories = [[None]*3]*50 # [joint_angles,trajectory_plan,trajectory_time]

			time_before = rospy.get_rostime()

			# Minimization performed 5 times
			for i in range (5):

				# Determine joint angles for minimization
				q = JointAnglesHandOffPose()


				# MoveIt! planned 10 times for each minimization result
				for j in range (10):

					# Checking to see if minimization problem was solved with a fixed max number of iterations
					if q[0] != 99:
						joints = {'left_s0': q[0], 'left_s1': q[1], 'left_e0': q[2], 'left_e1': q[3], 'left_w0': q[4], 'left_w1': q[5], 'left_w2': q[6], 'right_s0': q[7], 'right_s1': q[8], 'right_e0': q[9], 'right_e1': q[10], 'right_w0': q[11], 'right_w1': q[12], 'right_w2': q[13]}

						# Planning in MoveIt!
						group_both_arms.set_joint_value_target(joints)
						plan_both = group_both_arms.plan()

						# Checking to see if MoveIt! was able to plan a collision-free path
						if len(plan_both.joint_trajectory.joint_names) != 0:
							temp1 = plan_both
							temp2 = plan_both.joint_trajectory.points[len(plan_both.joint_trajectory.points)-1].time_from_start
						else:
							temp1 = 0
							temp2 = rospy.Time(10)
					else:
						temp1 = 0
						temp2 = rospy.Time(10)

					# Storing joint angles, trajectory plan, and trajectory time for comparison later
					Trajectories[10*i+j] = [q,temp1,temp2]

			time_after = rospy.get_rostime()
			print "\nMinimization and MoveIt! planning time: ", (time_after.secs-time_before.secs)+(time_after.nsecs-time_before.nsecs)/1e+9

			# Find trajectory that took the least amount of time and execute it
			traj_time = np.full(50,None)
			for i in range(50):
				temp3 = Trajectories[i][2]
				traj_time[i] = temp3.secs + temp3.nsecs/1e9

			minimum_time_entry =  np.nanargmin(traj_time)

			print "Least trajectory time: ", Trajectories[minimum_time_entry][2].secs + Trajectories[minimum_time_entry][2].nsecs/1e+9

			print "Executing least trajectory time."
			group_both_arms.execute(Trajectories[minimum_time_entry][1])

			first_flag = False





if __name__ == '__main__':
	main()

#!/usr/bin/env python

import sys
import copy
import rospy
import geometry_msgs.msg

from std_msgs.msg import (Header, String)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)

import moveit_commander
import moveit_msgs.msg

from moveit_commander import MoveGroupCommander



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

	# Initialize separation distance of Baxter's grippers for the hand off
	gripper_separation = 0.1

	# Average pose of left and right end-effectors to determine an initial starting position for the hand-off of the object
	pose_target_right = geometry_msgs.msg.Pose()
	pose_target_right.position.x = 0.5*(left_ee_pose.pose.position.x + right_ee_pose.pose.position.x) 
	pose_target_right.position.y = 0.5*(left_ee_pose.pose.position.y + right_ee_pose.pose.position.y) - gripper_separation
	pose_target_right.position.z = 0.5*(left_ee_pose.pose.position.z + right_ee_pose.pose.position.z)
	pose_target_right.orientation.x = 0.56508
	pose_target_right.orientation.y = -0.5198
	pose_target_right.orientation.z = -0.54332
	pose_target_right.orientation.w = -0.33955
	#pose_target_right.orientation.x = 0.5*(left_ee_pose.pose.orientation.x + right_ee_pose.pose.orientation.x)
	#pose_target_right.orientation.y = 0.5*(left_ee_pose.pose.orientation.y + right_ee_pose.pose.orientation.y)
	#pose_target_right.orientation.z = 0.5*(left_ee_pose.pose.orientation.z + right_ee_pose.pose.orientation.z)
	#pose_target_right.orientation.w = 0.5*(left_ee_pose.pose.orientation.w + right_ee_pose.pose.orientation.w)

	pose_target_left = geometry_msgs.msg.Pose()
	pose_target_left.position.x = 0.5*(left_ee_pose.pose.position.x + right_ee_pose.pose.position.x)
	pose_target_left.position.y = 0.5*(left_ee_pose.pose.position.y + right_ee_pose.pose.position.y) + gripper_separation
	pose_target_left.position.z = 0.5*(left_ee_pose.pose.position.z + right_ee_pose.pose.position.z)
	pose_target_left.orientation.x = 0.69283
	pose_target_left.orientation.y = 0.1977
	pose_target_left.orientation.z = -0.16912
	pose_target_left.orientation.w = 0.67253
	#pose_target_left.orientation.x = 0.5*(left_ee_pose.pose.orientation.x + right_ee_pose.pose.orientation.x)
	#pose_target_left.orientation.y = 0.5*(left_ee_pose.pose.orientation.y + right_ee_pose.pose.orientation.y)
	#pose_target_left.orientation.z = 0.5*(left_ee_pose.pose.orientation.z + right_ee_pose.pose.orientation.z)
	#pose_target_left.orientation.w = 0.5*(left_ee_pose.pose.orientation.w + right_ee_pose.pose.orientation.w)

	# Ensure position of the hand-off is in Baxter's work-space
	if pose_target_right.position.x < 0.2: 	# x-position is not inside Baxter
		pose_target_right.position.x = 0.2
		pose_target_left.position.x = 0.2

	# Set target poses for right_arm and left_arm groups
	#group_right_arm.set_pose_target(pose_target_right)
	#group_left_arm.set_pose_target(pose_target_left)



	# Generate plans for both_arms group and show if successful
	print "============ Generating plan for both arms with poses"
	print pose_target_right.position
	print pose_target_left.position
	group_both_arms.set_pose_target(pose_target_right, 'right_gripper')
	group_both_arms.set_pose_target(pose_target_left, 'left_gripper')

	plan_both = group_both_arms.plan()

	print plan_both

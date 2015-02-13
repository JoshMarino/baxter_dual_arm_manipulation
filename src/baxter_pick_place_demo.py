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
from moveit_msgs.msg import Grasp



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


	rospy.sleep(2)
	scene.remove_world_object("part")


	# Publish a demo scene
	p = PoseStamped()
	p.header.frame_id = robot.get_planning_frame()

	p.pose.position.x = 0.72651
	p.pose.position.y = -0.041037
	p.pose.position.z = 0.19097

	p.pose.orientation.x = 0.56508
	p.pose.orientation.y = -0.5198
	p.pose.orientation.z = -0.54332
	p.pose.orientation.w = -0.33955


	print "=============== Publishing a demo scene."
	scene.add_box("cube", p, (0.1, 0.1, 0.1))


	rospy.sleep(1)

##---------------------------------------------------------------------------------##
	# Pick an object using a specified grasp
	g = Grasp()


	# Configure grasp pose
	p = PoseStamped()
	p.header.frame_id = robot.get_planning_frame()
	p.pose.position.x = 0.32
	p.pose.position.y = -0.7
	p.pose.position.z = 0.5
	p.pose.orientation.x = 0
	p.pose.orientation.y = 0
	p.pose.orientation.z = 0
	p.pose.orientation.w = 1	

	g.grasp_pose = p


	# Configure pre-grasp approach
	g.pre_grasp_approach.direction.vector.x = 1.0
	g.pre_grasp_approach.direction.header.frame_id = robot.get_planning_frame()
	g.pre_grasp_approach.min_distance = 0.2
	g.pre_grasp_approach.desired_distance = 0.4


	# Configure post-grasp retreat
	g.post_grasp_retreat.direction.header.frame_id = robot.get_planning_frame()
	g.post_grasp_retreat.direction.vector.z = 1.0
	g.post_grasp_retreat.min_distance = 0.1
	g.post_grasp_retreat.desired_distance = 0.25


	# Set-up pre-grasp posture
	g.pre_grasp_posture.joint_names = "right_gripper_base"
	g.pre_grasp_posture.points = 1


	# Set-up grasp posture
	g.grasp_posture.joint_names = "right_gripper_base"
	g.grasp_posture.points = 0	


	# Pick up cube using right arm
	#grasps.push_back(g);
	group_right_arm.pick("cube",g)
##---------------------------------------------------------------------------------##

	rospy.spin()

#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
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
  rospy.sleep(30)
  print "============ Starting tutorial "

  #Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
  scene = moveit_commander.PlanningSceneInterface()

  #Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. In this case the group is the joints in the left arm. This interface can be used to plan and execute motions on the left arm.
  group_left_arm = moveit_commander.MoveGroupCommander("left_arm")
  group_right_arm = moveit_commander.MoveGroupCommander("right_arm")

  #We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)


  #--Getting Basic Information--#

  #We can get the name of the reference frame for this robot
  print "============ Reference frame: %s" % group_left_arm.get_planning_frame()

  #We can also print the name of the end-effector link for this group
  print "============ End-effector link: %s" % group_left_arm.get_end_effector_link()

  #We can get a list of all the groups in the robot
  print "============ Robot Groups:"
  print robot.get_group_names()

  #Sometimes for debugging it is useful to print the entire state of the robot.
  print "============ Printing robot state"
  print robot.get_current_state()
  print "============"




  #group = MoveGroupCommander("left_arm")

  print "============ Generating plan for left arm"
  pose_target_left = geometry_msgs.msg.Pose()
  pose_target_left.orientation.x = 0
  pose_target_left.orientation.y = 1.0
  pose_target_left.orientation.z = 0
  pose_target_left.orientation.w = 0
  pose_target_left.position.x = 0.75
  pose_target_left.position.y = 0.27
  pose_target_left.position.z = 0.15
  group_left_arm.set_pose_target(pose_target_left)

  plan_1eft = group_left_arm.plan()

  print "============ Generating plan for right arm"
  pose_target_right = geometry_msgs.msg.Pose()
  pose_target_right.orientation.x = 0
  pose_target_right.orientation.y = -1.0
  pose_target_right.orientation.z = 0
  pose_target_right.orientation.w = 0
  pose_target_right.position.x = -0.75
  pose_target_right.position.y = 0.27
  pose_target_right.position.z = 0.15
  group_right_arm.set_pose_target(pose_target_right)

  #Now, we call the planner to compute the plan and visualize it if successful. Note that we are just planning, not asking move_group to actually move the robot.

  plan_right = group_right_arm.plan()

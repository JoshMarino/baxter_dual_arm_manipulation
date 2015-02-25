#!/usr/bin/env python

import rospy
import numpy as np
import tf

from geometry_msgs.msg import Pose
from scipy.optimize import leastsq
from math import fabs, pi
from tf.transformations import euler_from_quaternion, quaternion_from_euler
# ===================================================================================================================================================== #



# ===================================================================================================================================================== #
# Setting up left arm initial position and converting quaternion to XYZ Euler angles for minimization
left_arm_start = geometry_msgs.msg.Pose()
left_arm_start.position.x = 0.657579481614
left_arm_start.position.y = 0.851981417433
left_arm_start.position.z = 0.0388352386502
left_arm_start.orientation.x = -0.366894936773
left_arm_start.orientation.y = 0.885980397775
left_arm_start.orientation.z = 0.108155782462
left_arm_start.orientation.w = 0.262162481772

quaternion_left_start = (left_arm_start.orientation.x, left_arm_start.orientation.y, left_arm_start.orientation.z, left_arm_start.orientation.w)
(left_arm_start_roll, left_arm_start_pitch, left_arm_start_yaw) = euler_from_quaternion(quaternion_left_start)


# Setting up right arm initial position and converting quaternion to XYZ Euler angles for minimization
right_arm_start = geometry_msgs.msg.Pose()
right_arm_start.position.x = 0.656982770038
right_arm_start.position.y = -0.852598021641
right_arm_start.position.z = 0.0388609422173
right_arm_start.orientation.x = 0.367048116303
right_arm_start.orientation.y = 0.885911751787
right_arm_start.orientation.z = -0.108908281936
right_arm_start.orientation.w = 0.261868353356

quaternion_right_start = (right_arm_start.orientation.x, right_arm_start.orientation.y, right_arm_start.orientation.z, right_arm_start.orientation.w)
(right_arm_start_roll, right_arm_start_pitch, right_arm_start_yaw) = euler_from_quaternion(quaternion_right_start)
# ===================================================================================================================================================== #



# ===================================================================================================================================================== #
# Minimization - Position
left_arm_start_pos_array = np.array([left_arm_start.position.x, left_arm_start.position.y, left_arm_start.position.z])
right_arm_start_pos_array = np.array([right_arm_start.position.x, right_arm_start.position.y, right_arm_start.position.z])

minimization_pos_function = lambda x: np.absolute(left_arm_start_pos_array - x) + np.absolute(x - right_arm_start_pos_array)

x0_pos_array = np.array([99,99,99])

hand_off_pos = leastsq(minimization_pos_function, x0_array)
hand_off_position = hand_off_pos[0]
print "Hand off position: ", hand_off_position


# Minimization - Euler XYZ angles
left_arm_start_euler_array = np.array([left_arm_start_roll, left_arm_start_pitch, left_arm_start_yaw])
right_arm_start_euler_array = np.array([right_arm_start_roll, right_arm_start_pitch, right_arm_start_yaw])

minimization_euler_function = lambda x: np.absolute(left_arm_start_euler_array - x) + np.absolute(x - right_arm_start_euler_array)

x0_euler_array = np.array([99,99,99])

hand_off_euler = leastsq(minimization_euler_function, x0_euler_array)
hand_off_orientation = hand_off_euler[0]
print "Hand off orientation, Euler XYZ angles: ", hand_off_orientation


# Converting Euler XYZ angles to quaternion, with the following constraints:
# 		1) left_arm_handoff_roll = right_arm_handoff_roll
#		2) left_arm_handoff_pitch = right_arm_handoff_pitch + math.pi/2
# 		3) left_arm_handoff_yaw = right_arm_handoff_yaw + math.pi
quaternion_left_handoff = quaternion_from_euler(hand_off_orientation[0], hand_off_orientation[1] + pi/4., hand_off_orientation[2] + pi/2.)
quaternion_right_handoff = quaternion_from_euler(hand_off_orientation[0], hand_off_orientation[1] - pi/4., hand_off_orientation[2] - pi/2.)
# ===================================================================================================================================================== #



# ===================================================================================================================================================== #
# Setting up left arm hand-off position and orientation
left_arm_handoff = geometry_msgs.msg.Pose()
left_arm_handoff.position.x = hand_off_position[0]
left_arm_handoff.position.y = hand_off_position[0] + 0.05
left_arm_handoff.position.z = hand_off_position[0]
left_arm_handoff.orientation.x = quaternion_left_handoff[0]
left_arm_handoff.orientation.y = quaternion_left_handoff[0]
left_arm_handoff.orientation.z = quaternion_left_handoff[0]
left_arm_handoff.orientation.w = quaternion_left_handoff[0]


# Setting up right arm hand-off position and orientation
right_arm_handoff = geometry_msgs.msg.Pose()
right_arm_handoff.position.x = hand_off_position[0]
right_arm_handoff.position.y = hand_off_position[0] - 0.05
right_arm_handoff.position.z = hand_off_position[0]
right_arm_handoff.orientation.x = quaternion_right_handoff[0]
right_arm_handoff.orientation.y = quaternion_right_handoff[0]
right_arm_handoff.orientation.z = quaternion_right_handoff[0]
right_arm_handoff.orientation.w = quaternion_right_handoff[0]
# ===================================================================================================================================================== #

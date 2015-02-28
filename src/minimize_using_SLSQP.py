import numpy as np
import rospy
import math



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




# Takes configuration variables in joint space and returns the end-effector position and Euler XYZ angles
def JS_to_P(q,arm):

	kin = baxter_kinematics(arm)
	FK = kin.forward_position_kinematics()

	Pose = Forward_Kinematics(q)

	x = Pose.position.x
	y = Pose.position.y
	z = Pose.position.z

	quaternion = (Pose.orientation.x, Pose.orientation.y, Pose.orientation.z, Pose.orientation.w)
	(roll, pitch, yaw) = euler_from_quaternion(quaternion)

	P = np.array([[x],[y],[z],[roll],[pitch],[yaw]])

	return P


# Normalize P1-P2: square root of sum of squares
def norm(P1, P2):

	normalized_result = math.sqrt( (P1[0,0]-P2[0,0])**2 + (P1[1,0]-P2[1,0])**2 + (P1[2,0]-P2[2,0])**2 + (P1[3,0]-P2[3,0])**2 + (P1[4,0]-P2[4,0])**2 + (P1[5,0]-P2[5,0])**2 )

	return normalized_result





# Minimization function to find a hand-off pose starting with initial right and left end-effector poses
minimization = lambda q: norm( JS_to_P(q[0:7,0],'left'), P_left_current ) + norm( JS_to_P(q[7:14,0],'right'), P_right_current )



def main():

	# Initialize node
    rospy.init_node('Minimization')

	# Start Moveit Commander
	InitializeMoveitCommander()

	# Obtain current poses of left and right end-effectors: [x,y,z,roll,pitch,yaw].T
	P_left_pose = group_left_arm.get_current_pose()
	P_right_pose = group_right_arm.get_current_pose()
	P_left_euler = group_left_arm.get_current_rpy()
	P_right_euler = group_right_arm.get_current_rpy()

	P_left_current = np.array([[P_left_pose.pose.position.x],[P_left_pose.pose.position.y],[P_left_pose.pose.position.z],[P_left_euler[0]],[P_left_euler[1]],[P_left_euler[2]]])
	P_right_current = np.array([[P_right_pose.pose.position.x],[P_right_pose.pose.position.y],[P_right_pose.pose.position.z],[P_right_euler[0]],[P_right_euler[1]],[P_right_euler[1]]])


	# Initial guess
	x0 = np.full((14,1), 99)

	# Bounds for SLSQP: e0, e1, s0, s1, w0, w1, w2
	bnds = ((-3.05417993878, 3.05417993878), (-0.05, 2.618), (-1.70167993878, 1.70167993878), (-2.147, 1.047), (-3.059, 3.059), (-1.57079632679, 2.094), (-3.059, 3.059), (-3.05417993878, 3.05417993878), (-0.05, 2.618), (-1.70167993878, 1.70167993878), (-2.147, 1.047), (-3.059, 3.059), (-1.57079632679, 2.094), (-3.059, 3.059)) # lower and upper bounds for each q (length 14): left, right

	# Constraint equality
	Handoff_separation = np.array([[0],[0.1],[0],[0],[math.pi/2.],[math.pi]])
	cons = ({'type': 'eq', 'fun': lambda q: JS_to_P(q[0:7,0],'left') - JS_to_P(q[7:14,0],'right') - Handoff_separation )


	# Minimization
	result = scipy.optimize.minimize(minimization, x0, method='SLSQP', bounds=bnds, constraints=cons, tol=0.001)

	print "Results of SLSQP Minimization: ", result

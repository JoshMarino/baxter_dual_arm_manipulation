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
import baxter_interface
import cv2
import cv_bridge
import rospkg
import os
import time

from scipy.optimize import minimize
from math import sqrt
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from tf.transformations import euler_from_matrix
from std_msgs.msg import (Header, String)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from moveit_commander import MoveGroupCommander
from baxter_core_msgs.msg import (DigitalIOState, EndEffectorState)
from baxter_interface import CHECK_VERSION
from sensor_msgs.msg import Image
from moveit_msgs.msg import RobotState, RobotTrajectory, DisplayTrajectory, DisplayRobotState, Constraints
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest, GetStateValidityResponse


# global first_flag, force_left_gripper, force_right_gripper
first_flag = False
force_left_gripper = False
force_right_gripper = False


# Initialize moveit commander and move group commander
def InitializeMoveitCommander():

    #First initialize moveit_commander 
    #print "============ Starting tutorial setup"
    moveit_commander.roscpp_initialize(sys.argv)

    #Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()

    #Wait for RVIZ to initialize. This sleep is ONLY to allow Rviz to come up.
    #print "============ Waiting for RVIZ..."
    rospy.sleep(1)
    #print "============ Starting tutorial "

    #Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
    global scene
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)

    # Add in objects
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.8
    p.pose.position.y = 0.0
    p.pose.position.z = -0.70
    scene.add_box("table", p, (0.762, 1.239, 1.0))


    #Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. In this case the group is the joints in the left arm. This interface can be used to plan and execute motions on the left arm.
    global group_both_arms, group_left_arm, group_right_arm
    group_both_arms = MoveGroupCommander("both_arms")
    group_both_arms.set_goal_position_tolerance(0.01)
    group_both_arms.set_goal_orientation_tolerance(0.01)
    group_both_arms.set_planning_time(5.0)
    group_both_arms.set_planner_id("SBLkConfigDefault")

    group_left_arm = MoveGroupCommander("left_arm")
    group_left_arm.set_goal_position_tolerance(0.01)
    group_left_arm.set_goal_orientation_tolerance(0.01)
    group_left_arm.set_planning_time(5.0)
    group_left_arm.set_planner_id("SBLkConfigDefault")

    group_right_arm = MoveGroupCommander("right_arm")
    group_right_arm.set_goal_position_tolerance(0.01)
    group_right_arm.set_goal_orientation_tolerance(0.01)
    group_right_arm.set_planning_time(5.0)
    group_right_arm.set_planner_id("SBLkConfigDefault")

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


    # Add collision object for the cube initially in the left gripper
    pose_left = PoseStamped()

    pose_left.header.frame_id = "left_gripper"
    pose_left.pose.position = Point(*[0,0,0.07])
    pose_left.pose.orientation = Quaternion(*[0,0,0,1])


    scene.remove_attached_object("left_gripper")
    rospy.sleep(1)
    scene.remove_world_object("cube")
    rospy.sleep(1)
    scene.attach_box("left_gripper", "cube", pose_left, (0.06,0.06,0.06))
    rospy.sleep(3)


    group_both_arms.attach_object("cube", "left_gripper")
    rospy.sleep(3)




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


    # Bounds for SLSQP: s0, s1, e0, e1, w0, w1, w2 (left,right)
    bnds = ( (-1.70167993878, 1.70167993878), (-2.147, 1.047), (-3.05417993878, 3.05417993878), (-0.05, 2.618), (-3.059, 3.059), (-1.57079632679, 2.094), (-3.059, 3.059),(-1.70167993878, 1.70167993878), (-2.147, 1.047), (-3.05417993878, 3.05417993878), (-0.05, 2.618),  (-3.059, 3.059), (-1.57079632679, 2.094), (-3.059, 3.059)) # lower and upper bounds for each q (length 14)


    # Initial guess
    x0 = np.array([[random.uniform(bnds[i][0]/2.,bnds[i][1]/2.)] for i in range(14)])

    # Constraint equality

    Handoff_separation = np.array([[0.0],[0.25],[0.0],[math.pi/1.],[0],[-math.pi/2.]])
    cons = ({'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[9,0]  - Handoff_separation[0,0]},                          #x-sep-distance = 0
            {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[10,0]  - Handoff_separation[2,0]},                         #y-sep-distance = 0
            {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[11,0]  - Handoff_separation[1,0]},                         #z-sep-distance = 0.12
            {'type': 'eq', 'fun': lambda q: math.fabs(JS_to_PrPlRrl(q)[6,0]) - Handoff_separation[3,0]},                #roll = pi
            {'type': 'eq', 'fun': lambda q: math.fabs(JS_to_PrPlRrl(q)[7,0]) - Handoff_separation[4,0]},                #pitch = 0
            {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[8,0] - Handoff_separation[5,0]},                           #yaw = -pi/2
            {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[0,0] - (P_left_current[0,0] + P_right_current[0,0])/2.},   #x_pos_right = midpoint of initial poses
            {'type': 'eq', 'fun': lambda q: JS_to_PrPlRrl(q)[1,0] - (P_left_current[1,0] + P_right_current[1,0])/2.})   #y_pos_right = midpoint of initial poses



    # Minimization
    result = minimize(minimization, x0, method='SLSQP', jac=jacobian, bounds=bnds, constraints=cons, tol=0.1, options={'maxiter': 40})

    print "\nNumber of iterations: \n", result.success, result.nit
    #print result

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
    global first_flag
    if msg.state and not first_flag:
        first_flag = True
    return



# Reads force sensor for left gripper to determine if holding block
def checkForceLeftGripper(msg):

    left_force = msg.force

    global force_left_gripper

    if left_force > 0.0:
        force_left_gripper = True
    else:
        force_left_gripper = False

    return


# Reads force sensor for left gripper to determine if holding block
def checkForceRightGripper(msg):

    right_force = msg.force

    global force_right_gripper

    if right_force > 0.0:
        force_right_gripper = True
    else:
        force_right_gripper = False

    return



# Send the image located at the specified path to the head display on Baxter.
def send_image(path):
    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)


def checkTrajectoryValidity(robot_trajectory, groups=[]):
    """Given a robot trajectory, deduce it's groups and check it's validity on each point of the traj
    returns True if valid, False otherwise
    It's considered not valid if any point is not valid"""
    rospy.wait_for_service('check_state_validity')
    get_state_validity = rospy.ServiceProxy('check_state_validity', GetStateValidity)
    #req = GetStateValidityRequest(rs,group)
    if len(groups) > 0:
        groups_to_check = groups
    else:
        groups_to_check = ['group_both_arms'] # Automagic group deduction... giving a group that includes everything
    for traj_point in robot_trajectory.joint_trajectory.points:
        rs = RobotState()
        rs.joint_state.name = robot_trajectory.joint_trajectory.joint_names
        rs.joint_state.position = traj_point.positions
        constraints = Constraints()
        for group in groups_to_check:
            req = GetStateValidityRequest(rs, group, constraints)
            resp = get_state_validity(req)
            print "\n",resp,"\n"
            if not resp.valid: # if one point is not valid, the trajectory is not valid
                #rospy.logerr("Trajectory is not valid at point (RobotState):" + str(rs) + "with result of StateValidity: " + str(resp))
                #rospy.logerr("published in /robot_collision_state the conflicting state")
                #drs = DisplayRobotState()
                #drs.state = rs
                #self.robot_state_collision_pub.publish(drs)
                return False
    fin_time = time.time()
    #rospy.logwarn("Trajectory validity of " + str(len(robot_trajectory.joint_trajectory.points)) + " points took " + str(fin_time - init_time))
    return True



# Main portion of code
def main():

    # Initialize node
    rospy.init_node('minimization_process')

    # Start Moveit Commander
    InitializeMoveitCommander()

    # Enable baxter
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()

    #Calibrate and open left gripper
    left_gripper = baxter_interface.Gripper('left')
    left_gripper.calibrate()
    left_gripper.open()
    gripper_identifier = "left"

    #Calibrate and open right gripper
    right_gripper = baxter_interface.Gripper('right')
    right_gripper.calibrate()
    right_gripper.open()

    # Create a subscriber to listen for a button press on Baxter which means to run the minimization process once.
    #rospy.Subscriber("/robot/digital_io/left_lower_button/state",DigitalIOState,checkButtonPress)
    #rospy.Subscriber("/robot/digital_io/right_lower_button/state",DigitalIOState,checkButtonPress)
    rospy.Subscriber("/robot/digital_io/left_itb_button1/state",DigitalIOState,checkButtonPress)
    rospy.Subscriber("/robot/digital_io/right_itb_button1/state",DigitalIOState,checkButtonPress)

    # Create a subscriber to determine if block was successfully transfered from one hand to the other
    rospy.Subscriber("/robot/end_effector/left_gripper/state",EndEffectorState,checkForceLeftGripper)
    rospy.Subscriber("/robot/end_effector/right_gripper/state",EndEffectorState,checkForceRightGripper)

    # Initialization of KDL Kinematics for right and left grippers
    rospack = rospkg.RosPack()
    pkgpath = rospack.get_path('baxter_dual_arm_manipulation')
    SUBDIR_URDF = "src/baxter.urdf"
    DIR_URDF = os.path.join(pkgpath, SUBDIR_URDF)

    robot = URDF.from_xml_file(DIR_URDF)

    global kdl_kin_left, kdl_kin_right
    kdl_kin_left = KDLKinematics(robot, "base", "left_gripper")
    kdl_kin_right = KDLKinematics(robot, "base", "right_gripper")

    # Locations of images to be sent to head display
    SUBDIR_IM1 = "move_arms1.png"
    SUBDIR_IM2 = "computing_handoff_pose1.png"
    SUBDIR_IM3 = "moving_arms1.png"
    SUBDIR_IM4 = "no_trajectory_found.png"
    DIR_IM1 = os.path.join(pkgpath, SUBDIR_IM1)
    DIR_IM2 = os.path.join(pkgpath, SUBDIR_IM2)
    DIR_IM3 = os.path.join(pkgpath, SUBDIR_IM3)
    DIR_IM4 = os.path.join(pkgpath, SUBDIR_IM4)

    first_image = DIR_IM1
    second_image = DIR_IM2
    third_image = DIR_IM3
    fourth_image = DIR_IM4


    # Continuously be listening for the flag to indicate to run the minimization process once
    global first_flag
    while not rospy.is_shutdown():

        # Image telling to press the button to start minimization process
        send_image(first_image)

        if first_flag == True:

            print "Entered minimization process."
            print "Computing handoff poses..."
            send_image(second_image)

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

            if Trajectories[minimum_time_entry][1] != 0:

                q_before = group_both_arms.get_current_joint_values()

                send_image(third_image)

                print "Least trajectory time: ", Trajectories[minimum_time_entry][2].secs + Trajectories[minimum_time_entry][2].nsecs/1e+9

                print "Checking trajectory for validity: ", checkTrajectoryValidity(Trajectories[minimum_time_entry][1])

                print "Executing least trajectory time."
                group_both_arms.execute(Trajectories[minimum_time_entry][1])


                # Change planner to smaller longest_valid_segment_fraction and plan to hand-off pose
                group_both_arms.set_planner_id("SBLkConfigSmall")
                group_left_arm.set_planner_id("SBLkConfigSmall")
                group_right_arm.set_planner_id("SBLkConfigSmall")
                group_both_arms.set_planning_time(5.0)
                group_left_arm.set_planning_time(5.0)
                group_right_arm.set_planning_time(5.0)
                #rospy.sleep(1.5)

                P = JS_to_PrPlRrl(Trajectories[minimum_time_entry][0])  #np.array([[x_left],[y_left],[z_left],[x_right],[y_right],[z_right],[roll],[pitch],[yaw],[sep_dist_x],[sep_dist_y],[sep_dist_z]])
                P_left = JS_to_P(Trajectories[minimum_time_entry][0][0:7],'left') #np.array([[x],[y],[z],[roll],[pitch],[yaw]])
                P_right = JS_to_P(Trajectories[minimum_time_entry][0][7:14],'right') #np.array([[x],[y],[z],[roll],[pitch],[yaw]])

                x_dist = P[0,0] - P[3,0]
                y_dist = P[1,0] - P[4,0]
                z_dist = P[2,0] - P[5,0]

                left_x = P[0,0] - (12.0/25.0)*x_dist/2.0
                right_x = P[3,0] + (12.0/25.0)*x_dist/2.0

                left_y = P[1,0] - (12.0/25.0)*y_dist/2.0
                right_y = P[4,0] + (12.0/25.0)*y_dist/2.0

                left_z = P[2,0] - (12.0/25.0)*z_dist/2.0
                right_z = P[5,0] + (12.0/25.0)*z_dist/2.0

                Trajectories1 = [[None]*2]*5 # [joint_angles,trajectory_plan,trajectory_time]

                for i in range(2):
                    group_both_arms.set_pose_target([left_x,left_y,left_z,P_left[3,0],P_left[4,0],P_left[5,0]],"left_gripper")
                    group_both_arms.set_pose_target([right_x,right_y,right_z,P_right[3,0],P_right[4,0],P_right[5,0]],"right_gripper")

                    plan_both = group_both_arms.plan()

                    while len(plan_both.joint_trajectory.joint_names)==0:
                        group_both_arms.set_pose_target([left_x,left_y,left_z,P_left[3,0],P_left[4,0],P_left[5,0]],"left_gripper")
                        group_both_arms.set_pose_target([right_x,right_y,right_z,P_right[3,0],P_right[4,0],P_right[5,0]],"right_gripper")
                        plan_both = group_both_arms.plan()


                    if len(plan_both.joint_trajectory.joint_names) != 0:
                        temp1 = plan_both
                        temp2 = plan_both.joint_trajectory.points[len(plan_both.joint_trajectory.points)-1].time_from_start
                    else:
                        temp1 = 0
                        temp2 = rospy.Time(10)

                    # Storing trajectory plan, and trajectory time for comparison later
                    Trajectories1[i] = [temp1,temp2]

                # Find trajectory that took the least amount of time and execute it
                traj_time1 = np.full(2,None)
                for i in range(2):
                    temp3 = Trajectories1[i][1]
                    traj_time1[i] = temp3.secs + temp3.nsecs/1e9
                print "Time for moving closer to each other:", traj_time1

                minimum_time_entry = np.nanargmin(traj_time1)

                print "Checking trajectory for validity: ", checkTrajectoryValidity(Trajectories1[minimum_time_entry][0])

                group_both_arms.execute(Trajectories1[minimum_time_entry][0])

                # Set planners back to default
                group_both_arms.set_planner_id("SBLkConfigDefault")
                group_left_arm.set_planner_id("SBLkConfigDefault")
                group_right_arm.set_planner_id("SBLkConfigDefault")
                group_both_arms.set_planning_time(5.0)
                group_left_arm.set_planning_time(5.0)
                group_right_arm.set_planning_time(5.0)



                # Create poses for cube to be seen in RViz using MoveIt!
                pose_left = PoseStamped()
                pose_right = PoseStamped()

                pose_left.header.frame_id = "left_gripper"
                pose_left.pose.position = Point(*[0,0,0.07])
                pose_left.pose.orientation = Quaternion(*[0,0,0,1])

                pose_right.header.frame_id = "right_gripper"
                pose_right.pose.position = Point(*[0,0,0.07])
                pose_right.pose.orientation = Quaternion(*[0,0,0,1])


                # Swap cube from left to right grippers
                if gripper_identifier == "left":
                    # Close right gripper, open left
                    right_gripper.close()
                    rospy.sleep(1)
                    if force_right_gripper == True:
                        left_gripper.open()

                    # Remove collision object from left gripper and place on right gripper
                    group_both_arms.detach_object("left_gripper")
                    rospy.sleep(1)
                    group_both_arms.attach_object("cube", "right_gripper")
                    rospy.sleep(1)

                    # Move cube to right gripper for visualization purposes
                    scene.remove_attached_object("left_gripper")
                    rospy.sleep(1)
                    scene.remove_world_object("cube")
                    rospy.sleep(1)
                    scene.attach_box("right_gripper", "cube", pose_right, (0.06,0.06,0.06))
                    rospy.sleep(1)

                    # Change identifier to be in right gripper now
                    gripper_identifier = "right"

                # Swap cube from right to left gripper
                elif gripper_identifier == "right":
                    # Close left gripper, open right
                    left_gripper.close()
                    rospy.sleep(1)
                    if force_left_gripper == True:
                        right_gripper.open()

                    # Remove collision object from right gripper and place on left gripper
                    group_both_arms.detach_object("right_gripper")
                    rospy.sleep(1)
                    group_both_arms.attach_object("cube", "left_gripper")
                    rospy.sleep(1)

                    # Move cube to left gripper for visualization purposes
                    scene.remove_attached_object("right_gripper")
                    rospy.sleep(1)
                    scene.remove_world_object("cube")
                    rospy.sleep(1)
                    scene.attach_box("left_gripper", "cube", pose_left, (0.06,0.06,0.06))
                    rospy.sleep(1)

                    # Change identifier to be in left gripper now
                    gripper_identifier = "left"


                # Move both arms back to initial poses
                group_both_arms.set_joint_value_target(q_before)

                plan_both = group_both_arms.plan()
                while len(plan_both.joint_trajectory.joint_names) == 0:
                    plan_both = group_both_arms.plan()

                group_both_arms.execute(plan_both)


                print "Press button to do again."

            else:

                send_image(fourth_image)
                rospy.sleep(5)


            # Specify that minimization process has completed
            first_flag = False





if __name__ == '__main__':
    main()

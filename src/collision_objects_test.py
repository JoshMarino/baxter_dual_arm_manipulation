#!/usr/bin/env python

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
import tf.transformations as tr
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
# Baxter imports
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_core_msgs.msg import DigitalIOState
# moveit stuff:
import moveit_commander
import moveit_msgs.msg
from moveit_commander import MoveGroupCommander

# non ROS stuff:
import numpy as np

# import set joint goals and SE(3) goals:
import joint_targets as jt
# import cartesian_targets as ct


class MoveItCollisionTest( object ):
    def __init__(self):
        rospy.loginfo("Creating MoveItCollisionTest object")

        # instantiate baxter interfaces:
        self.right_arm = baxter_interface.limb.Limb("right")
        self.left_arm = baxter_interface.limb.Limb("left")

        # enable Baxter if needed:
        rospy.loginfo("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        rospy.loginfo("Enabling robot... ")
        self._rs.enable()

        # set clean shutdown hook
        rospy.on_shutdown(self.clean_shutdown)


        # let's create MoveIt! objects:
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.right_arm_group = moveit_commander.MoveGroupCommander("right_arm")
        self.left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
        self.both_arm_group = moveit_commander.MoveGroupCommander("both_arms")
        # self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

        # add displays for the start and goal states:
        self.start_state_pub = rospy.Publisher("/move_group/start_state", moveit_msgs.msg.DisplayRobotState, queue_size=1)
        self.end_state_pub = rospy.Publisher("/move_group/end_state", moveit_msgs.msg.DisplayRobotState, queue_size=1)

        rospy.sleep(3.0)

        # let's add world collision objects:
        self.world_collisions()

        # now we can plan and go:
        self.both_arm_group.set_goal_position_tolerance(0.01)
        self.both_arm_group.set_goal_orientation_tolerance(0.01)
        self.both_arm_group.set_planning_time(5.0)

        self.both_arm_group.set_joint_value_target(jt.targets['open'])
        rospy.loginfo("Attempting to plan")
        plan = self.both_arm_group.plan()
        rospy.loginfo("Done with planning")

        # move the arms there:
        self.both_arm_group.go()

        # now create a subscriber for the button presses to decide if we should re-plan
        self.button_sub = rospy.Subscriber("/robot/digital_io/left_lower_button/state", DigitalIOState,
                                           self.button_cb, queue_size=1)
        
        # rospy.loginfo("Sleeping...")
        # rospy.sleep(5)
        # rospy.loginfo("Done sleeping")


        # while not rospy.is_shutdown():
        #     q = self.both_arm_group.get_random_joint_values()
        #     self.both_arm_group.set_joint_value_target(q)
        #     plan = self.both_arm_group.plan()

        #     if not plan:
        #         rospy.logwarn("No plan found")
        #         continue

        #     # rospy.loginfo("Executing...")
        #     # self.both_arm_group.go()
        #     # rospy.loginfo("Done executing")

        #     self.both_arm_group.set_joint_value_target(jt.targets['grasp'])
        #     rospy.loginfo("Attempting to plan")
        #     plan = self.both_arm_group.plan()
        #     rospy.loginfo("Done with planning")

        #     rospy.loginfo("Sleeping...")
        #     rospy.sleep(5)
        #     rospy.loginfo("Done sleeping")

        #     # rospy.loginfo("Executing...")
        #     # self.both_arm_group.go()
        #     # rospy.loginfo("Done executing")
        
        return

    def button_cb(self, button_state):
        if button_state.state:
            self.random_plan_to_grasp(move=False)
        return


    def random_plan_to_grasp(self, move=False):
        qrand = self.both_arm_group.get_random_joint_values()

        for i in range(5):
            # display start state:
            start_state = moveit_msgs.msg.RobotState()
            jss = JointState()
            jss.name = self.both_arm_group.get_active_joints()
            jss.position = qrand
            start_state.joint_state = jss
            dstart_state = moveit_msgs.msg.DisplayRobotState()
            dstart_state.state = start_state
            self.start_state_pub.publish(dstart_state)
            # display goal state:
            end_state = moveit_msgs.msg.RobotState()
            jse = JointState()
            jse.name,jse.position = zip(*[(key, val) for key,val in jt.targets['grasp'].items()])
            end_state.joint_state = jse
            dend_state = moveit_msgs.msg.DisplayRobotState()
            dend_state.state = end_state
            self.end_state_pub.publish(dend_state)
        # plan and move
        self.both_arm_group.set_start_state(start_state)
        self.both_arm_group.set_joint_value_target(jt.targets['grasp'])
        plan = self.both_arm_group.plan()
        if not plan:
            rospy.logwarn("No plan found")
            return plan
        if move:
            rospy.loginfo("Executing...")
            self.both_arm_group.go()
            rospy.loginfo("Done executing")
        return plan
        
    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self.reset_control_modes()
        # self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True


    def reset_control_modes(self):
        rate = rospy.Rate(100)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self.right_arm.exit_control_mode()
            self.left_arm.exit_control_mode()
            rate.sleep()
        return True


    def world_collisions(self):
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.pose.position.x = 0.8
        p.pose.position.y = 0.025
        p.pose.position.z = -0.6
        self.scene.add_box("table", p, (0.75, 1.25, 0.68))

        return



def main():
    rospy.init_node("moveit_collision_test", log_level=rospy.INFO)
    
    try:
        colltest = MoveItCollisionTest()
    except rospy.ROSInterruptException: pass

    rospy.spin()



if __name__ == '__main__':
	main()

#! /usr/bin/env python

# Author: Nuha Nishat
# Date: 1/30/20

# Edited by Akshaya Agrawal
# Date: 05/25/21

'''
    The Door is placed at (-0.24, 0, 0) position. Initially the Handle is located at
    (-0.24,-0.36, 0) -> (x,y,z).
    The path of the door is such that it follows a circle centered at (0,-0.5).
    The Height of the handle from the base is 0.35
'''

import rospy
import sys, os
import math
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf, math
import tf.transformations
import pdb
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String

from moveit_commander.conversions import pose_to_list
from moveit_msgs.msg import RobotState, PlanningScene, PlanningSceneComponents, AllowedCollisionEntry, \
    AllowedCollisionMatrix
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
import time
import numpy as np
import math
import copy
from tf.transformations import quaternion_from_euler

# move_group_python_interface_tutorial was used as reference

class MoveRobot():
    def __init__(self):
        # Initialize moveit commander and ros node for moveit

        # To read from redirected ROS Topic
        joint_state_topic = ['joint_states:=/j2s7s300/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        rospy.init_node('move-kinova', anonymous=False)
        moveit_commander.roscpp_initialize(sys.argv)

        # Define robot using RobotCommander. Provided robot info such as
        # kinematic model and current joint state
        self.robot = moveit_commander.RobotCommander()

        # Setting the world
        self.scene = moveit_commander.PlanningSceneInterface()

        # Define the planning group for the arm you are using
        # You can easily look it up on rviz under the MotionPlanning tab
        self.move_group = moveit_commander.MoveGroupCommander("arm")
        self.move_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Set the precision of the robot
        rospy.set_param('/move_group/trajectory_execution/allowed_start_tolerance', 0.3)

        rospy.wait_for_service("/apply_planning_scene", 10.0)
        rospy.wait_for_service("/get_planning_scene", 10.0)

        self.apply_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        self.get_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        rospy.sleep(2)

        # To see the trajectory
        self.disp = moveit_msgs.msg.DisplayTrajectory()

        self.disp.trajectory_start = self.robot.get_current_state()

        self.rate = rospy.Rate(10)

        self.move_group.allow_replanning(1)

        self.main()

    def set_planner_type(self, planner_name):
        if planner_name == "RRT":
            self.move_group.set_planner_id("RRTConnectkConfigDefault")
        if planner_name == "RRT*":
            self.move_group.set_planner_id("RRTstarkConfigDefault")
        if planner_name == "PRM*":
            self.move_group.set_planner_id("PRMstarkConfigDefault")

    def go_to_joint_state(self, joint_state):
        joint_goal = JointState()
        joint_goal.position = joint_state
        self.move_group.set_joint_value_target(joint_goal.position)

        self.plan = self.move_group.plan()
        self.move_group.go(wait=True)
        self.move_group.execute(self.plan, wait=True)

        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.sleep(2)

    def go_to_goal(self, ee_pose):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = ee_pose[0]
        pose_goal.position.y = ee_pose[1]
        pose_goal.position.z = ee_pose[2]

        if len(ee_pose) == 6:
            quat = tf.transformations.quaternion_from_euler(math.radians(ee_pose[3]), math.radians(ee_pose[4]),
                                                            math.radians(ee_pose[5]))
            pose_goal.orientation.x = quat[0]
            pose_goal.orientation.y = quat[1]
            pose_goal.orientation.z = quat[2]
            pose_goal.orientation.w = quat[3]

        else:
            pose_goal.orientation.x = ee_pose[3]
            pose_goal.orientation.y = ee_pose[4]
            pose_goal.orientation.z = ee_pose[5]
            pose_goal.orientation.w = ee_pose[6]

        self.move_group.set_pose_target(pose_goal)
        self.move_group.set_planning_time(20)
        rospy.sleep(2)
        self.move_group.go(wait=True)
        self.move_group.stop()

        self.move_group.clear_pose_targets()
        rospy.sleep(2)

    def move_gripper(self, cmd):
        if cmd == "Close":
            self.move_gripper.set_named_target("Close")
        elif cmd == "Open":
            self.move_gripper.set_named_target("Open")
        else:
            self.move_gripper.set_joint_value_target(cmd)
        self.move_gripper.go(wait=True)
        rospy.sleep(2)

    def display_trajectory(self):
        self.disp_pub = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory,
                                        queue_size=20)
        self.disp.trajectory.append(self.plan)
        print(self.disp.trajectory)
        self.disp_pub.publish(self.disp)

    def go_to_finger_state(self, cmd):
        if cmd == "Close":
            self.move_gripper.set_named_target("Close")
        elif cmd == "CurlClose":
            self.move_gripper.set_named_target("CurlClose")
        elif cmd == "Open":
            self.move_gripper.set_named_target("Open")
        else:
            """gripper_goal = JointState()
            gripper_goal.position = cmd"""
            self.move_gripper.set_joint_value_target(cmd)
        # self.plan_gripper = self.move_gripper.plan()

        self.move_gripper.go(wait=True)
        """self.move_gripper.execute(self.plan_gripper, wait=True)"""
        self.move_gripper.stop()
        self.move_gripper.clear_pose_targets()
        rospy.sleep(2)

    def go_to_finger_joint_state(self, joint_values):
        try:
            gripper_states = JointState()
            gripper_states.position = joint_values
            self.move_gripper.set_joint_value_target(gripper_states.position)
            self.move_gripper.go(wait=True)
            """self.move_gripper.execute(self.plan_gripper, wait=True)"""
            self.move_gripper.stop()
            self.move_gripper.clear_pose_targets()
            rospy.sleep(2)
            return True
        except:
            return False

    def go_to_arm_joint_state(self, joint_values):
        try:
            arm_states = JointState()
            arm_states.position = joint_values
            # try:
            self.move_group.set_joint_value_target(arm_states.position)
            # except Exception as e:
            #     rospy.loginfo(e)
            self.move_group.go(wait=True)
            """self.move_gripper.execute(self.plan_gripper, wait=True)"""
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            rospy.sleep(2)
            return True
        except:
            return False

    def main(self):

        # Set up path here

        # Pick planner
        self.set_planner_type("RRT")



        ############## Grab the handle ###################

        # Open the Gripper
        rospy.loginfo('opening the gripper')
        self.go_to_finger_state('Open')

        # Go to handle
        rospy.loginfo('Going to palm link position')
        val = -0.359
        for i in range (0,2):
            val = val - i*0.001
            # rospy.loginfo('Going to palm link position')
            # rospy.loginfo(val)
            self.go_to_goal([-0.24, val, 0.35, 90, 180, 0])



        ############# Going to finger link position ###################
        rospy.loginfo('Going to finger link position')
        val_r = 1.12
        val_l = 1.1
        for i in range(0, 6):
            val_r = val_r + i * 0.005
            val_l = val_l + i * 0.005
            # rospy.loginfo('Going to finger link position')
            # rospy.loginfo(val_r)
            # rospy.loginfo(val_l)
            self.go_to_finger_joint_state([val_r,val_l,val_l])



        # # ### Try to open door the door
        # val_x = -0.24
        # val_y = -0.359
        # for i in range(0,10):
        #     val_x = val_x + 0.001
        #     for j in range(0,10):
        #         val_y = val_y + 0.001
        #         rospy.loginfo('try opening door')
        #         rospy.loginfo(val_x)
        #         rospy.loginfo(val_y)
        #         self.go_to_goal([val_x, val_y, 0.35, 90, 180, 0])
        #     val_y = -0.359
        #

        ###### Get Waypoints ###########


        ###### Plan path ##########
        waypoints = []

        thetas = []
        t = 30.0

        # First plan
        for i in range(0, 2):  # In current scene max value of theta is 46
            t = t + 2
            thetas.append((t / 180.0) * np.pi)

        # rospy.loginfo('current pose: ')
        wpose = self.move_group.get_current_pose().pose
        # rospy.loginfo(wpose)

        for val in thetas:
            w1 = cal_new_pos(val)
            wpose.position.x = w1[0]
            wpose.position.y = w1[1]
            waypoints.append(copy.deepcopy(wpose))

        # rospy.loginfo('waypoints: ', waypoints)

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.001,  # eef_step
            0.0)  # jump_threshold
        rospy.loginfo('First plan Executing')
        self.move_group.execute(plan, wait=True)

        rospy.sleep(2)
        # Change Orientation

        wpose = self.move_group.get_current_pose().pose
        rospy.loginfo('Changing Orientation')
        q = quaternion_from_euler(np.pi/2, np.pi, -np.pi/20)
        self.go_to_goal([wpose.position.x,wpose.position.y,wpose.position.z,q[0],q[1],q[2],q[3]])
        # rospy.loginfo('new pose: ')

        rospy.sleep(2)
        wpose = self.move_group.get_current_pose().pose
        # rospy.loginfo(wpose)

        # Second plan and execution

        waypoints = []

        thetas = []
        t = 34.0
        for i in range(0, 2):  # Max 8
            t = t + 2
            thetas.append((t / 180.0) * np.pi)

        # rospy.loginfo('current pose: ')
        wpose = self.move_group.get_current_pose().pose
        # rospy.loginfo(wpose)
        for val in thetas:
            w1 = cal_new_pos(val)
            wpose.position.x = w1[0]
            wpose.position.y = w1[1]
            waypoints.append(copy.deepcopy(wpose))

        # rospy.loginfo('waypoints: ', waypoints)

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.001,  # eef_step
            0.0)  # jump_threshold

        rospy.loginfo('Second plan executing')
        self.move_group.execute(plan, wait=True)

        rospy.loginfo('Final pose: ')
        wpose = self.move_group.get_current_pose().pose
        rospy.loginfo(wpose)

        rospy.spin()

def cal_new_pos(theta):
    circle_c = [0, -0.5]
    pt1 = [-0.24, -0.36]
    circle_r = round(math.sqrt(((pt1[0] - circle_c[0])** 2) + ((pt1[1] - circle_c[1])** 2)),3)
    # print(circle_r)
    x = round(-circle_r*math.cos(theta),3)
    y = round(circle_c[1] + circle_r*math.sin(theta),3)
    return [x,y]

if __name__ == '__main__':
    MoveRobot()
#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import copy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Vector3
from tf.transformations import quaternion_from_euler

######################################
# GLOBAL
######################################

def euler_to_quaternion(role, pitch, yaw):
    q = quaternion_from_euler(role, pitch, yaw)
    return Quaternion(q[0], q[1], q[2], q[3])

######################################
# MAIN
######################################

def main():

    # init node
    rospy.init_node("moveit_tutorial_node")

    # configuration for moveit
    robot = moveit_commander.RobotCommander()

    # group name
    rarm = moveit_commander.MoveGroupCommander("rarm")
    larm = moveit_commander.MoveGroupCommander("larm")
    upper = moveit_commander.MoveGroupCommander("upper_body")
    rarm_waist = moveit_commander.MoveGroupCommander("rarm_with_waist")

    ## reset pose
    joint_goal = upper.get_current_joint_values()
    for i in range(0,len(joint_goal)):
      joint_goal[i] = 0
    joint_goal[6] = -3
    joint_goal[16] = -3
    upper.go(joint_goal, wait=True)

    ## pose goal with rarm
    rarm_waist.set_end_effector_link("r_eef_pick_link")
    pose_goal = Pose()
    pose_goal.orientation = euler_to_quaternion(-1.57,0,0)
    pose_goal.position.x = 0.8
    pose_goal.position.y = -0.2
    pose_goal.position.z = 1.3
    rarm_waist.set_pose_target(pose_goal)
    rarm_waist.go(wait=True)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

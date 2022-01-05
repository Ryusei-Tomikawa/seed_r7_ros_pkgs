#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import copy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Vector3

######################################
# MAIN
######################################

def main():

    # init node
    rospy.init_node("moveit_tutorial_node")

    # configuration for moveit
    robot = moveit_commander.RobotCommander()
    # display of group name (rarm,larm,upper_body...
    print ""
    print "Robot groups:", robot.get_group_names()
    # display of robot current state
    print ""
    print "Robot state:", robot.get_current_state()

    # group name
    rhand = moveit_commander.MoveGroupCommander("rhand")
    rarm = moveit_commander.MoveGroupCommander("rarm")
    larm = moveit_commander.MoveGroupCommander("larm")
    upper = moveit_commander.MoveGroupCommander("upper_body")
    rarm_waist = moveit_commander.MoveGroupCommander("rarm_with_waist")
    larm_waist = moveit_commander.MoveGroupCommander("larm_with_waist")

    # display of joint name
    print ""
    print "RHAND Joint names:", robot.get_joint_names("rhand")
    print ""
    print "RARM Joint names:", robot.get_joint_names("rarm")
    print ""
    print "LARM Joint names:", robot.get_joint_names("larm")
    print ""
    print "RARM_WITH_WAIST Joint names:", robot.get_joint_names("rarm_with_waist")
    print ""
    print "LARM_WITH_WAIST Joint names:", robot.get_joint_names("larm_with_waist")
    print ""
    print "UPPER_BODY Joint names:", robot.get_joint_names("upper_body")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

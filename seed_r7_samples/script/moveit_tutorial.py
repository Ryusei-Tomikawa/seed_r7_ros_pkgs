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

    ## joint goal with rarm
    # create joint goal
    joint_goal = rarm.get_current_joint_values()
    
    # elbow is 90[deg]
    joint_goal[1] = -0.7
    joint_goal[2] = -0.7
    joint_goal[3] = -1.57
    joint_goal[4] = -0.5
        
    # plan and excute
    print "RARM",rarm.go(joint_goal, wait=True)
    
    # display of joint values
    print ""
    print "RARM Joint values:",rarm.get_current_joint_values()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

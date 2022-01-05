#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import copy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Vector3
from std_msgs.msg import *

######################################
# MAIN
######################################

Flag = Bool()
# configuration for moveit
robot = moveit_commander.RobotCommander()

# group name
rarm = moveit_commander.MoveGroupCommander("rarm")
larm = moveit_commander.MoveGroupCommander("larm")
upper = moveit_commander.MoveGroupCommander("upper_body")
rarm_waist = moveit_commander.MoveGroupCommander("rarm_with_waist")

def FlagCallback(flag):

    Flag.data = flag.data
    print("flag:=", Flag.data)

# def main():
    # ## reset pose
    # joint_goal = upper.get_current_joint_values()
    # for i in range(0,len(joint_goal)):
    #   joint_goal[i] = 0
    # joint_goal[6] = -3
    # joint_goal[16] = -3
    # # upper.go(joint_goal, wait=True)  

    ## joint goal with rarm
    # # create joint goal
    # joint_goal = rarm.get_current_joint_values()
    # # elbow is 90[deg]
    # joint_goal[3] = -1.57
    # # plan and excute
    # print("RARM",rarm.go(joint_goal, wait=True))

    # ## joint goal with larm
    # larm.set_end_effector_link("l_eef_pick_link")
    # joint_goal = larm.get_current_joint_values()
    # # elbow is 90[deg]
    # joint_goal[3] = -1.57
    # print("LARM",larm.go(joint_goal, wait=True))

    ## joint goal with rarm-with-waist
    #joint_goal = rarm_waist.get_current_joint_values()
    # waist yaw
    #joint_goal[0] = 1.0
    # elbow
    #joint_goal[6] = -1.0
    #print "RARM_WITH_WAIST",rarm_waist.go(joint_goal, wait=True)

    ## joint goal with upper-body
    joint_goal = upper.get_current_joint_values()
    # waist yaw
    joint_goal[0] = 1.0
    # both arm's elbow
    joint_goal[6] = -2.3
    joint_goal[16] = -2.3
    # print("UPPER_BODY",upper.go(joint_goal, wait=True))

    if Flag.data == False:
        upper.go(joint_goal, wait=True)
    elif Flag.data == True:
        print(" Flag_data := true")


if __name__ == '__main__':
    # init node
    rospy.init_node("moveit_tutorial_node")

    #sub
    rospy.Subscriber('flag', Bool, FlagCallback)

    print("wait for flag ...")

    # rate = rospy.Rate(10.0)
    # while not rospy.is_shutdown():
    #     try:
    #         main()
    #         # break
    #     except rospy.ROSInterruptException:
    #         pass
    #     rate.sleep()
    rospy.spin()

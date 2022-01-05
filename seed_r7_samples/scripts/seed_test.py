#! /usr/bin/env python
# -*- coding: utf-8 -*-

from genpy import message
import rospy
import math
import copy
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import *
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Vector3
from rsdlab_fcsc_pkg.srv import*

Flag = Bool()

# configuration for moveit
robot = moveit_commander.RobotCommander()

# group name
rarm = moveit_commander.MoveGroupCommander("rarm")
larm = moveit_commander.MoveGroupCommander("larm")
upper = moveit_commander.MoveGroupCommander("upper_body")
rarm_waist = moveit_commander.MoveGroupCommander("rarm_with_waist")


def FlagCallback(flag_):
    
    Flag.data = flag_.data

    string_pub = rospy.Publisher('trajectory_execution_event', String, queue_size=100)
    r = rospy.Rate(10)
    
    message = String()
    message.data = "stop"

    while not rospy.is_shutdown():
        if flag_.data == True:
            string_pub.publish(message)
            break
        else: # flag_.data == False
            print("Go Manipulation!")
            break


def main():

    ## reset pose
    joint_goal = upper.get_current_joint_values()
    for i in range(0,len(joint_goal)):
        joint_goal[i] = 0
    joint_goal[6] = -3
    joint_goal[16] = -3
    if Flag.data == False:
        upper.go(joint_goal, wait=True)  

    ## pose goal with rarm
    rarm.set_end_effector_link("r_eef_pick_link")
    pose_goal = Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = -0.25
    pose_goal.position.z = 1.2
    rarm.set_pose_target(pose_goal)
    # print("RARM",rarm.go(wait=True))
    if Flag.data == False:
        print("RARM",rarm.go(wait=True))
        print("")
        print("RARM Pose:",rarm.get_current_pose().pose)


if __name__ == '__main__':

     # init node
    rospy.init_node("moveit_tutorial_node")

    rospy.Subscriber("person_flag", Bool, FlagCallback)

    rate = rospy.Rate(10.0)
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
    rate.sleep()
    rospy.spin()

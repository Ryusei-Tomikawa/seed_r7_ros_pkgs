#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import copy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Vector3
from seed_r7_ros_controller.srv import*

######################################
# MAIN
######################################

def main():

    # init node
    rospy.init_node("moveit_tutorial_node")

    # configuration for moveit
    robot = moveit_commander.RobotCommander()

    rospy.wait_for_service('/seed_r7_ros_controller/hand_control')
    service = rospy.ServiceProxy('/seed_r7_ros_controller/hand_control', HandControl)

    # group name
    rarm = moveit_commander.MoveGroupCommander("rarm")
    larm = moveit_commander.MoveGroupCommander("larm")
    upper = moveit_commander.MoveGroupCommander("upper_body")
    rarm_waist = moveit_commander.MoveGroupCommander("rarm_with_waist")
    larm_waist = moveit_commander.MoveGroupCommander("larm_with_waist")


    ## reset pose
    joint_goal = upper.get_current_joint_values()
    for i in range(0,len(joint_goal)):
      joint_goal[i] = 0
    joint_goal[6] = -3 #left
    joint_goal[16] = -3 #right
    upper.go(joint_goal, wait=True)  

    # miss???
    # joint_goal = upper.get_current_joint_values()
    # for i in range(0,len(joint_goal)):
    #   joint_goal[i] = 0
    # joint_goal[0] = 1.57
    # joint_goal[6] = -3 #left
    # joint_goal[16] = -3 #right
    # upper.go(joint_goal, wait=True)

    # rospy.sleep(3)

    # joint_goal = upper.get_current_joint_values()
    # for i in range(0,len(joint_goal)):
    #   joint_goal[i] = 0
    # joint_goal[6] = -3 #left
    # joint_goal[16] = -3 #right
    # upper.go(joint_goal, wait=True)


    # release
    # response = service(1,'grasp',100)
    # rospy.sleep(6)
    # response = service(1,'release',100)
    # rospy.sleep(6)

    ## joint goal with rarm
    # create joint goal
    #joint_goal = rarm.get_current_joint_values()

    # waist yaw and r_wrist yaw 1.57 turn
    joint_goal = rarm_waist.get_current_joint_values()
    for i in range(0,len(joint_goal)):
      joint_goal[i] = 0
    joint_goal[0] = 1.57 #waist yaw
    joint_goal[6] = -3
    joint_goal[7] = 1.57
    # rarm_waist.go(joint_goal, wait=True)  

    joint_goal = larm_waist.get_current_joint_values()
    for i in range(0,len(joint_goal)):
      joint_goal[i] = 0
    joint_goal[0] = 1.57 #waist yaw
    joint_goal[6] = -3
    joint_goal[7] = 1.57

    rarm_waist.go(joint_goal, wait=True)
    larm_waist.go(joint_goal, wait=True)
    
    # elbow is 90[deg]
    #joint_goal[1] = -0.7
    #joint_goal[2] = -0.7
    #joint_goal[3] = -1.57
    #joint_goal[4] = -0.5
        
    # plan and excute
    # print ("RARM",upper.go(joint_goal, wait=True))
    
    # display of joint values
    #print ("")
    #print ("RARM Joint values:",rarm.get_current_joint_values())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

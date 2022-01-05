#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import *
import math
import copy
import moveit_commander
import moveit_msgs.msg

from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Vector3
from tf.transformations import quaternion_from_euler

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

######################################
# GLOBAL
######################################

count = 0
person_ = Bool()
SLEEP_TIME = 5.0

waypoint = [
    [(-1.646,0.495,0.0),(0.0,0.0,0.0,0.927)],
    [(-3.789,-5.8807,0.0),(0.0,0.0,0.696004,0.718038)],
    [(1.39363,-5.32839,0.0),(0.0,0.0,0.679585,0.733597)],
    [(1.5932,5.36782,0.0),(0.0,0.0,0.704773,0.709432)],
    [(-3.6035,6.08306,0.0),(0.0,0.0,-0.0369427,0.999317)],
    [(-1.646,0.495,0.0),(0.0,0.0,0.0,0.927)]
]

def goal_pose(pose):
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

def personCallback(person):
    person_.data = person.data

def main():
    
    global count

    rospy.loginfo("OK")
    rospy.loginfo("wait signal person_flag...")

    # rospy.spin()
     # actionlib client declaration 
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    client.wait_for_server()

    for pose in waypoint:
            goal = goal_pose(pose)
            client.send_goal(goal)
            if person_.data:
                client.cancel_goal()
            else:
                succeeded = client.wait_for_result(rospy.Duration(30))
                if succeeded:
                    rospy.loginfo("OK")
                    count += 1
                    print("count:= ", count)
                else:
                    rospy.loginfo("NG")
                if count == 3:
                    client.cancel_goal()
                    state = client.get_state()
                    print("Goal Cansel!")
                    rospy.loginfo('state:=%d', state)
                    count = 0

    
# -----------------------------------------------------------------------------
    
if __name__ == '__main__':

    rospy.init_node("final_task_node")

    person_sub = rospy.Subscriber('person_detect', Bool, personCallback)
    
    rate = rospy.Rate(10.0)
    try:
        while not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
    rate.sleep()

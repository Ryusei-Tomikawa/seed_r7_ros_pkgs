#! /usr/bin/env python
# -*- coding: utf-8 -*-

from os import name, wait
import rospy
import math
import copy
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import *
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Vector3
from tf.transformations import quaternion_from_euler

######################################
# GLOBAL
######################################

SLEEP_TIME = 1.5

def euler_to_quaternion(role, pitch, yaw):
    q = quaternion_from_euler(role, pitch, yaw)
    return Quaternion(q[0], q[1], q[2], q[3])

######################################
# MAIN
######################################]
def box_pose(x, y, z, ow, ox, oy, oz):
    box_pose = PoseStamped()
    box_pose.header.frame_id = "base_link"
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z
    box_pose.pose.orientation.w = ow
    box_pose.pose.orientation.x = ox
    box_pose.pose.orientation.y = oy
    box_pose.pose.orientation.z = oz

    return box_pose

def main():

    # init node
    rospy.init_node("moveit_tutorial_node")

    # configuration for moveit
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # group name
    rarm = moveit_commander.MoveGroupCommander("rarm")
    larm = moveit_commander.MoveGroupCommander("larm")
    upper = moveit_commander.MoveGroupCommander("upper_body")
    rarm_waist = moveit_commander.MoveGroupCommander("rarm_with_waist")
    larm_waist = moveit_commander.MoveGroupCommander("larm_with_waist")
    lifter = moveit_commander.MoveGroupCommander("lifter")

    rarm_waist_end_effector_link = rarm_waist.get_end_effector_link()
    larm_waist_end_effector_link = larm_waist.get_end_effector_link()
    rarm_waist.set_end_effector_link(rarm_waist_end_effector_link)
    larm_waist.set_end_effector_link(larm_waist_end_effector_link)

    reference_frame = "/base_link"

    upper.set_pose_reference_frame(reference_frame)
    rarm_waist.set_pose_reference_frame(reference_frame)
    larm_waist.set_pose_reference_frame(reference_frame)
    lifter.set_pose_reference_frame(reference_frame)

    ## reset pose
    joint_goal = upper.get_current_joint_values()
    for i in range(0,len(joint_goal)):
      joint_goal[i] = 0
    joint_goal[6] = -3
    joint_goal[16] = -3
    upper.go(joint_goal, wait=True)

    joint_goal[0] = 1.57
    upper.go(joint_goal, wait=True)

    ## pose goal with rarm
    rarm_waist.set_end_effector_link("r_eef_pick_link")
    pose_goal = Pose()
    pose_goal.orientation = euler_to_quaternion(-1.57,0.79,0)
    pose_goal.position.x = 0.242632
    pose_goal.position.y = 0.409981
    pose_goal.position.z = 1.169837
    rarm_waist.set_pose_target(pose_goal, rarm_waist_end_effector_link)
    rarm_waist.go(wait=True)
    rospy.sleep(SLEEP_TIME)

    ## reset pose
    joint_goal = upper.get_current_joint_values()
    for i in range(0,len(joint_goal)):
      joint_goal[i] = 0
    joint_goal[6] = -3
    joint_goal[16] = -3
    upper.go(joint_goal, wait=True)

    larm_waist.set_end_effector_link("l_eef_pick_link")
    pose_goal = Pose()
    pose_goal.orientation = euler_to_quaternion(0,0,0)
    pose_goal.position.x = 0.8
    pose_goal.position.y = 0.2
    pose_goal.position.z = 1.3
    larm_waist.set_pose_target(pose_goal)
    larm_waist.go(wait=True)
    rospy.sleep(SLEEP_TIME)
    # pose_goal.orientation = euler_to_quaternion(-1.54,0.58,1.53)
    # pose_goal.position.x = 0.24
    # pose_goal.position.y = 0.42
    # pose_goal.position.z = 1.22

    ## box add
    box_name = "box"
    box_pose_ = PoseStamped()
    box_pose_.header.frame_id = "r_eef_pick_link"
    box_pose_.pose.orientation = euler_to_quaternion(0,0,0)
    scene.add_box(box_name, box_pose_, size=(0.03, 0.03, 0.03))
    scene.add_box("shelf1", box_pose(0.0,0.8,0.44, 0.70,0,0,0.70), size=(0.8, 1.0, 0.01))
    scene.add_box("shelf2", box_pose(0.0,0.8,0.84, 0.70,0,0,0.70), size=(0.8, 1.0, 0.01))
    scene.add_box("shelf3", box_pose(0.0,0.8,1.2, 0.70,0,0,0.70), size=(0.8, 1.0, 0.01))
    scene.add_box("shelf4", box_pose(0.0,0.8,1.55, 0.70,0,0,0.70), size=(0.8, 1.0, 0.01))

    scene.add_box("wall1", box_pose(0.5,1.2,0.75, 0.70,0,0,0.70), size=(0.6, 0.01, 1.7))
    scene.add_box("wall2", box_pose(-0.5,1.2,0.75, 0.70,0,0,0.70), size=(0.6, 0.01, 1.7))
    rospy.sleep(SLEEP_TIME)

    ## pose goal with rarm
    # pose_goal.orientation = euler_to_quaternion(-1.57,0,0)
    # pose_goal.position.x = 0.6
    # pose_goal.position.y = -0.1
    # pose_goal.position.z = 1.2
    # rarm_waist.set_pose_target(pose_goal, rarm_waist_end_effector_link)
    # rarm_waist.go(wait=True)

    ## box attach
    grasping_group = 'rarm_with_waist'
    touch_links = robot.get_link_names(group=grasping_group)
    eef_link = rarm_waist.get_end_effector_link()
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    rospy.sleep(SLEEP_TIME)

    ## box release
    scene.remove_attached_object(eef_link, name=box_name)
    rospy.sleep(SLEEP_TIME)

    ## reset pose
    joint_goal = upper.get_current_joint_values()
    for i in range(0,len(joint_goal)):
      joint_goal[i] = 0
    joint_goal[6] = -3
    joint_goal[16] = -3
    upper.go(joint_goal, wait=True)

    ## box remove
    scene.remove_world_object(box_name)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

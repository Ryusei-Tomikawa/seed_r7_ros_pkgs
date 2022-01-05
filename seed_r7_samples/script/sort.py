#! /usr/bin/env python
# -*- coding: utf-8 -*-

from os import name, wait
import rospy
import math
import copy
from rospy.impl.tcpros_service import wait_for_service
import tf
import moveit_commander
import moveit_msgs.msg
from std_msgs.msg import *
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Vector3
from tf import transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler 

######################################
# GLOBAL
######################################

SLEEP_TIME = 1.5

sucessful = False 

def euler_to_quaternion(role, pitch, yaw):
    q = quaternion_from_euler(role, pitch, yaw)
    return Quaternion(q[0], q[1], q[2], q[3])

def quaternion_to_euler(quaternion):
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

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

def armgo(trans):
# def main():

    global sucessful

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
    waist = moveit_commander.MoveGroupCommander("waist")

    rarm_waist_end_effector_link = rarm_waist.get_end_effector_link()
    larm_waist_end_effector_link = larm_waist.get_end_effector_link()
    rarm_waist.set_end_effector_link(rarm_waist_end_effector_link)
    larm_waist.set_end_effector_link(larm_waist_end_effector_link)

    reference_frame = "/base_link"

    upper.set_pose_reference_frame(reference_frame)
    rarm_waist.set_pose_reference_frame(reference_frame)
    larm_waist.set_pose_reference_frame(reference_frame)
    lifter.set_pose_reference_frame(reference_frame)
    waist.set_pose_reference_frame(reference_frame)

    print("Robot groups:", robot.get_group_names())
    print("wait groups joint:", robot.get_joint_names("waist"))

    ## reset pose
    joint_goal = upper.get_current_joint_values()
    for i in range(0,len(joint_goal)):
      joint_goal[i] = 0
    joint_goal[6] = -3
    joint_goal[17] = -3
    upper.go(joint_goal, wait=True)

    waist_joint = waist.get_current_joint_values()
    for i in range(0, len(waist_joint)):
        waist_joint[i] = 0
    waist_joint[0] = 1.57
    waist.go(waist_joint, wait=True)

    lifter_goal = lifter.get_current_joint_values()
    print("lifter get current state :=", lifter_goal)
    for i in range(len(lifter_goal)):
        lifter_goal[i] = 0
    lifter_goal[0] = 1.570131778717041
    lifter_goal[1] = -1.570131778717041
    lifter_goal[2] = -1.5704814195632935
    lifter_goal[3] = 1.5704814195632935
    # lifter_goal[0] = 0.950131778717041
    # lifter_goal[1] = -0.950131778717041
    # lifter_goal[2] = -0.9504814195632935
    # lifter_goal[3] = 0.9504814195632935
    lifter.set_max_velocity_scaling_factor(0.8)
    lifter.go(lifter_goal, wait=True)
    rospy.sleep(3.0)
    print('lifter down 2 step')

    ## box add
    # box_name = "box"
    # box_pose_ = PoseStamped()
    # box_pose_.header.frame_id = "r_eef_pick_link"
    # box_pose_.pose.orientation = euler_to_quaternion(0,0,0)
    # scene.add_box(box_name, box_pose_, size=(0.03, 0.03, 0.03))
    scene.add_box("shelf1", box_pose(0.0,0.8,0.44, 0.70,0,0,0.70), size=(0.8, 1.0, 0.01))
    # scene.add_box("shelf2", box_pose(0.0,0.8,0.84, 0.70,0,0,0.70), size=(0.8, 1.0, 0.01))
    # scene.add_box("shelf3", box_pose(0.0,0.8,1.2, 0.70,0,0,0.70), size=(0.8, 1.0, 0.01))
    # scene.add_box("shelf4", box_pose(0.0,0.8,1.55, 0.70,0,0,0.70), size=(0.8, 1.0, 0.01))

    scene.add_box("wall1", box_pose(0.5,1.2,0.75, 0.70,0,0,0.70), size=(0.6, 0.01, 1.7))
    scene.add_box("wall2", box_pose(-0.5,1.2,0.75, 0.70,0,0,0.70), size=(0.6, 0.01, 1.7))
    rospy.sleep(SLEEP_TIME)

    if trans[0] > 0:
        print("Right Hand")
        ## pose goal with rarm
        rarm_waist.set_end_effector_link("r_eef_pick_link")
        pose_goal = Pose()
        # pose_goal.orientation = euler_to_quaternion(-1.57,0,0)
        pose_goal.orientation = euler_to_quaternion(-1.54,0.58,1.53)
        # pose_goal.orientation.w = -0.353047
        # pose_goal.orientation.x = 0.623361
        # pose_goal.orientation.y = 0.316346
        # pose_goal.orientation.z = -0.621855
        pose_goal.position.x = trans[0]
        pose_goal.position.y = trans[1]
        pose_goal.position.z = trans[2] + 0.05
        rarm_waist.set_pose_target(pose_goal, rarm_waist_end_effector_link)
        rarm_waist.go(wait=True)
        rospy.sleep(SLEEP_TIME)

        pose_goal.position.z = trans[2]
        rarm_waist.set_pose_target(pose_goal, rarm_waist_end_effector_link)
        lifter.set_max_velocity_scaling_factor(0.1)
        rarm_waist.go(wait=True)
        rospy.sleep(SLEEP_TIME*3)

        pose_goal.position.z = trans[2] + 0.05
        rarm_waist.set_pose_target(pose_goal, rarm_waist_end_effector_link)
        lifter.set_max_velocity_scaling_factor(0.1)
        rarm_waist.go(wait=True)
        rospy.sleep(SLEEP_TIME*3)

        waist_joint = waist.get_current_joint_values()
        for i in range(0, len(waist_joint)):
            waist_joint[i] = 0
        waist.set_max_velocity_scaling_factor(0.1)
        waist.go(waist_joint, wait=True)

        ## reset pose
        joint_goal = upper.get_current_joint_values()
        for i in range(0,len(joint_goal)):
            joint_goal[i] = 0
        joint_goal[6] = -3
        joint_goal[17] = -3
        upper.go(joint_goal, wait=True)

        sucessful = True


    
    elif trans[0] < 0:
        print("Left hand")

        ## pose goal with rarm
        larm_waist.set_end_effector_link("l_eef_pick_link")
        pose_goal = Pose()
        pose_goal.orientation = euler_to_quaternion(1.74, -0.025, 1.70)
        pose_goal.position.x = trans[0]
        pose_goal.position.y = trans[1]
        pose_goal.position.z = trans[2] + 0.1
        larm_waist.set_pose_target(pose_goal, larm_waist_end_effector_link)
        larm_waist.go(wait=True)
        rospy.sleep(SLEEP_TIME)

        pose_goal.position.z = trans[2] 
        larm_waist.set_pose_target(pose_goal, larm_waist_end_effector_link)
        larm_waist.go(wait=True)
        rospy.sleep(SLEEP_TIME*3)

        pose_goal.position.z = trans[2] + 0.1
        larm_waist.set_pose_target(pose_goal, larm_waist_end_effector_link)
        larm_waist.go(wait=True)
        rospy.sleep(SLEEP_TIME)

        waist_joint = waist.get_current_joint_values()
        for i in range(0, len(waist_joint)):
            waist_joint[i] = 0
        waist.set_max_velocity_scaling_factor(0.1)
        waist.go(waist_joint, wait=True)

        ## reset pose
        joint_goal = upper.get_current_joint_values()
        for i in range(0,len(joint_goal)):
            joint_goal[i] = 0
        joint_goal[6] = -3
        joint_goal[17] = -3
        upper.go(joint_goal, wait=True)

        sucessful = True


    scene.remove_world_object("shelf1")
    scene.remove_world_object("shelf2")
    scene.remove_world_object("shelf3")
    scene.remove_world_object("shelf4")
    scene.remove_world_object("wall1")
    scene.remove_world_object("wall2")

# def lifter_move(self, step):
#         ## reset pose
#         upper_goal = self.upper.get_current_joint_values()
#         for i in range(len(upper_goal)):
#             upper_goal[i] = 0
#         upper_goal[6] = -3
#         upper_goal[16] = -3
#         self.upper.go(upper_goal, wait=True)

#         if step == 3:
#             lifter_goal = self.lifter.get_current_joint_values()
#             print("lifter get current state :=", lifter_goal)
#             for i in range(len(lifter_goal)):
#                 lifter_goal[i] = 0
#             lifter_goal[0] = 1.570131778717041
#             lifter_goal[1] = -1.570131778717041
#             lifter_goal[2] = -1.5704814195632935
#             lifter_goal[3] = 1.5704814195632935
#             self.lifter.set_max_velocity_scaling_factor(0.8)
#             self.lifter.go(lifter_goal, wait=True)
#             rospy.sleep(2.0)
#             print('lifter down 3 step')

#         elif step == 2:
#             lifter_goal = self.lifter.get_current_joint_values()
#             print("lifter get current state :=", lifter_goal)
#             for i in range(len(lifter_goal)):
#                 lifter_goal[i] = 0
#             lifter_goal[0] = 0.950131778717041
#             lifter_goal[1] = -0.950131778717041
#             lifter_goal[2] = -0.9504814195632935
#             lifter_goal[3] = 0.9504814195632935
#             self.lifter.set_max_velocity_scaling_factor(0.8)
#             self.lifter.go(lifter_goal, wait=True)
#             rospy.sleep(2.0)
#             print('lifter down 2 step')

#         elif step == 1:
#             lifter_goal = self.lifter.get_current_joint_values()
#             print("lifter get current state :=", lifter_goal)
#             for i in range(len(lifter_goal)):
#                 lifter_goal[i] = 0
#             self.lifter.set_max_velocity_scaling_factor(0.8)
#             self.lifter.go(lifter_goal, wait=True)
#             rospy.sleep(2.0)
#             print('lifter down 1 step')

if __name__ == '__main__':

    global sucessful 

    rpy = [0,0,0]

    # init node
    rospy.init_node("moveit_tutorial_node")
    
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # (r_trans_riceball1,rot) = listener.lookupTransform('/base_link', '/riceball1_link', rospy.Time(0))
            # (l_trans_riceball2,rot) = listener.lookupTransform('/base_link', '/riceball2_link', rospy.Time(0))
            # (r_trans_hamburg_steak,rot) = listener.lookupTransform('/base_link', '/hamburg_steak_link', rospy.Time(0))
            # (l_trans_sandwich,rot) = listener.lookupTransform('/base_link', '/sandwich_link', rospy.Time(0))
            (r_trans_pached_lunch,rot) = listener.lookupTransform('/base_link', '/packed_lunch_link', rospy.Time(0))
            (l_trans_caffe_latte,rot) = listener.lookupTransform('/base_link', '/caffe_latte_link', rospy.Time(0))
            (trans,camera_rot) = listener.lookupTransform('/base_link', '/cam_1_link', rospy.Time(0))
            # rospy.loginfo("r_trans[0] :=%3f", r_trans_pached_lunch[0])
            # rospy.loginfo("r_trans[1] :=%3f", r_trans_pached_lunch[1])
            # rospy.loginfo("r_trans[2] :=%3f", r_trans_pached_lunch[2])
            # rospy.loginfo("l_trans[0] :=%3f", l_trans_caffe_latte[0])
            # rospy.loginfo("l_trans[1] :=%3f", l_trans_caffe_latte[1])
            # rospy.loginfo("l_trans[2] :=%3f", l_trans_caffe_latte[2])
            quaternion = Quaternion()
            quaternion.x = camera_rot[0]
            quaternion.y = camera_rot[1]
            quaternion.z = camera_rot[2]
            quaternion.w = camera_rot[3]
            rpy = Vector3()
            rpy = quaternion_to_euler(quaternion)
            role = rpy.x
            pitch = rpy.y
            yaw = rpy.z
            print("role:=", role)
            print("pitch:=", pitch)
            print("yaw:=", yaw)

            if(r_trans_pached_lunch[0] != 0 and l_trans_caffe_latte[0] != 0):
                armgo(r_trans_pached_lunch)
            if sucessful == True:
                armgo(l_trans_caffe_latte)
                break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    # try:
    #     main()
    # except rospy.ROSInterruptException:
    #     pass

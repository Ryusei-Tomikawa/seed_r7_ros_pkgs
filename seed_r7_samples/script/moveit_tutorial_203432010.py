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
SLEEP_TIME = 1.5
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
    scene = moveit_commander.PlanningSceneInterface()
    
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
    upper.stop()

    
    ## joint goal with rarm ##
    # create joint goal
    joint_goal = rarm.get_current_joint_values()
    
    # elbow is 90[deg]
    joint_goal[3] = -1.57
        
    # plan and excute
    print "RARM",rarm.go(joint_goal, wait=True)

    ##joint goal with larm
    #larm.set_end_effector_link("l_eef_pick_link")  # ryusei
    joint_goal=larm.get_current_joint_values()

    #elbow is 90[deg]
    joint_goal[3]=-1.57
    print "LARM",larm.go(joint_goal, wait=True)

    ## joint goal with rarm-with-waist
    joint_goal = rarm_waist.get_current_joint_values()
    
    # waist yaw
    joint_goal[0] = 1.0

    # elbow
    joint_goal[6] = -1.0
    print "RARM_WITH_WAIST",rarm_waist.go(joint_goal, wait=True)
    
    ## joint goal with upper-body
    joint_goal = upper.get_current_joint_values()
    
    # waist yaw
    joint_goal[0] = 0
    
    # both arm's elbow
    joint_goal[6] = -2.3
    joint_goal[16] = -2.3
    print "UPPER_BODY",upper.go(joint_goal, wait=True)
    
    
    ## pose goal with rarm
    rarm.set_end_effector_link("r_eef_pick_link")
    pose_goal = Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = -0.25
    pose_goal.position.z = 1.2
    rarm.set_pose_target(pose_goal)
    print "RARM",rarm.go(wait=True)
    print ""
    print "RARM Pose:",rarm.get_current_pose().pose
    
    ## pose goal with rarm_with_waist
    wrarm_waist.set_end_effector_link("r_eef_pick_link")
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.8
    pose_goal.position.y = -0.2
    pose_goal.position.z = 1.2
    rarm_waist.set_pose_target(pose_goal)
    print "RARM_WITH_WAIST",rarm_waist.go(wait=True)
    

    ## pose goal with cartesian path
    waypoints = []
    wpose = rarm_waist.get_current_pose().pose
    wpose.position.z += -1 * 0.1
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.y += 1 * 0.1
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.z += 1 * 0.1
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.y += -1 * 0.1
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = rarm_waist.compute_cartesian_path(waypoints,0.01,0.0)
    print "CARTESIAN",rarm_waist.execute(plan, wait=True)

    
    ## pose goal with rarm
    rarm_waist.set_end_effector_link("r_eef_pick_link")
    pose_goal = Pose()
    pose_goal.orientation = euler_to_quaternion(-1.57,0,0)
    pose_goal.position.x = 0.8
    pose_goal.position.y = -0.2
    pose_goal.position.z = 1.3
    rarm_waist.set_pose_target(pose_goal)
    rarm_waist.go(wait=True)
    
    ## box add
    box_name = "box"
    box_pose = PoseStamped()
    box_pose.header.frame_id = "r_eef_pick_link"
    box_pose.pose.orientation = euler_to_quaternion(0,0,0)
    scene.add_box(box_name, box_pose, size=(0.03, 0.03, 0.03))
    rospy.sleep(SLEEP_TIME)
    
    ## box attach
    grasping_group = 'rarm_with_waist'
    touch_links = robot.get_link_names(group=grasping_group)
    eef_link = rarm_waist.get_end_effector_link()
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    rospy.sleep(SLEEP_TIME)

    ## pose goal with rarm
    pose_goal.orientation = euler_to_quaternion(-1.57,0,0)
    pose_goal.position.x = 0.6
    pose_goal.position.y = -0.1
    pose_goal.position.z = 1.2
    rarm_waist.set_pose_target(pose_goal)
    rarm_waist.go(wait=True)
    
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

    
    # display of joint values
    print ""
    print "RARM Joint values:",rarm.get_current_joint_values()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

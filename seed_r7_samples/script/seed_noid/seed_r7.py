#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import copy
import actionlib
import moveit_commander
import moveit_msgs.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Vector3
from tf.transformations import quaternion_from_euler

######################################
# GLOBAL
######################################

init_points = [
    [(-3.0,-3.0,0.0),(0.0,0.0,0.0,1.0)]
]

waypoint = [
    [(-0.94,-0.02,0.0),(0.0,0.0,0.0,1.0)]
]

SLEEP_TIME = 2.0
def euler_to_quaternion(role, pitch, yaw):
    q = quaternion_from_euler(role, pitch, yaw)
    return Quaternion(q[0], q[1], q[2], q[3])

######################################
# MAIN
######################################
def main():
     # init node
    rospy.init_node("task_node")
    
    # actionlib client declaration 
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # waypoint
    for pose in waypoint:
        goal = goal_pose(pose)
        client.send_goal(goal)
        succeeded = client.wait_for_result(rospy.Duration(15))
        state = client.get_state()

        if succeeded:
            rospy.loginfo("Succeeded!")
        else:
            rospy.loginfo("Failed...")

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

    
    ################################################
    # pick and place
    ################################################
    ## pose goal with rarm
    rarm_waist.set_end_effector_link("r_eef_pick_link")
    pose_goal = Pose()
    pose_goal.orientation = euler_to_quaternion(-1.57,0,0)
    pose_goal.position.x = -0.7  
    pose_goal.position.y = -0.3  
    pose_goal.position.z = 1.2
    rarm_waist.set_start_state_to_current_state()
    rarm_waist.set_pose_target(pose_goal)
    rarm_waist.go(wait=True)
    print "RARM_WAIST Pose:",rarm_waist.get_current_pose().pose
    
    
    ## box add
    box_name = "box"
    box_pose = PoseStamped()
    box_pose.header.frame_id = "r_eef_pick_link"
    box_pose.pose.orientation = euler_to_quaternion(0,0,0)
    scene.add_box(box_name, box_pose, size=(0.05, 0.05, 0.05))
    rospy.sleep(SLEEP_TIME)
    
    ## box attach
    grasping_group = 'rarm_with_waist'
    touch_links = robot.get_link_names(group=grasping_group)
    eef_link = rarm_waist.get_end_effector_link()
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    rospy.sleep(SLEEP_TIME)
        
    ## reset pose
    joint_goal = upper.get_current_joint_values()
    for i in range(0,len(joint_goal)):
        joint_goal[i] = 0
    joint_goal[6] = -2.0
    joint_goal[16] = -2.0
    upper.go(joint_goal, wait=True)
    
    
    ###############################################
    # Triangle rarm
    ###############################################
    ##pose goal with rarm
    rarm.set_start_state_to_current_state()
    rarm.set_end_effector_link("r_eef_pick_link")
    pose_goal = Pose()
    pose_goal.orientation = euler_to_quaternion(-1.57,0,0)
    pose_goal.position.x = -0.5 
    pose_goal.position.y = -0.3 
    pose_goal.position.z = 1.3
    rarm.set_pose_target(pose_goal)
    print "RARM",rarm.go(wait=True)
    print ""
    print "RARM Pose:",rarm.get_current_pose().pose
        
    ## pose goal with rarm_with_waist
    rarm_waist.set_start_state_to_current_state()
    rarm_waist.set_end_effector_link("r_eef_pick_link")
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = -0.4  #-0.4
    pose_goal.position.y = -0.25 #-0.25
    pose_goal.position.z = 1.2
    rarm_waist.set_pose_target(pose_goal)
    print "RARM_WITH_WAIST",rarm_waist.go(wait=True)
        
    ## pose goal with cartesian path
    waypoints = []
    wpose = rarm_waist.get_current_pose().pose
    wpose.position.z += -1 * 0.1
    wpose.position.y += 1 * 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += -1 * 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += 1 * 0.1
    wpose.position.z += 1 * 0.1
    waypoints.append(copy.deepcopy(wpose))
 
    (plan, fraction) = rarm_waist.compute_cartesian_path(waypoints,0.005,0.0)
    print "CARTESIAN",rarm_waist.execute(plan, wait=True)
    
    ## pose goal with rarm
    pose_goal.orientation = euler_to_quaternion(-1.57,0,0)
    pose_goal.position.x = -0.5
    pose_goal.position.y = -0.5 
    pose_goal.position.z = 1.3  
    rarm_waist.set_pose_target(pose_goal)
    rarm_waist.go(wait=True)
    print "RARM_WAIST Pose:",rarm_waist.get_current_pose().pose
       
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

    # init_pose 
    for init_pose in init_points:
        init = back_pose(init_pose)
        client.send_goal(init)
        succeeded = client.wait_for_result(rospy.Duration(15))
        state = client.get_state()

        if succeeded:
            rospy.loginfo("Succeeded!")
        else:
            rospy.loginfo("Failed...")

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

def back_pose(init_pose):
    Goal_pose = MoveBaseGoal()
    Goal_pose.target_pose.header.frame_id = 'map'
    Goal_pose.target_pose.pose.position.x = init_pose[0][0]
    Goal_pose.target_pose.pose.position.y = init_pose[0][1]
    Goal_pose.target_pose.pose.position.z = init_pose[0][2]
    Goal_pose.target_pose.pose.orientation.x = init_pose[1][0]
    Goal_pose.target_pose.pose.orientation.y = init_pose[1][1]
    Goal_pose.target_pose.pose.orientation.z = init_pose[1][2]
    Goal_pose.target_pose.pose.orientation.w = init_pose[1][3]

    return Goal_pose

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

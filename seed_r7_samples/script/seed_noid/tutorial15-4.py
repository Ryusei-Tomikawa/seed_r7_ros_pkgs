#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import math

import rospy
import actionlib
import moveit_commander

from geometry_msgs.msg import Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

######################################
# GLOBAL
######################################
SLEEP_TIME = 1.5
waypoints = [
	[(0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)],
	[(-1.852, 3.324, 0), (0, 0, 0.7070, 0.7070)],
	[(0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)]
]


def euler_to_quaternion(role, pitch, yaw):
	q = quaternion_from_euler(role, pitch, yaw)
	return Quaternion(q[0], q[1], q[2], q[3])


def goal_pose(pose):
	goal_pose = MoveBaseGoal()
	goal_pose.target_pose.header.frame_id = "map"
	goal_pose.target_pose.pose.position.x = pose[0][0]
	goal_pose.target_pose.pose.position.y = pose[0][1]
	goal_pose.target_pose.pose.position.z = pose[0][2]
	goal_pose.target_pose.pose.orientation.x = pose[1][0]
	goal_pose.target_pose.pose.orientation.y = pose[1][1]
	goal_pose.target_pose.pose.orientation.z = pose[1][2]
	goal_pose.target_pose.pose.orientation.w = pose[1][3]

	return goal_pose


def pour():
	# configuration for moveit
	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()

	# group name
	upper = moveit_commander.MoveGroupCommander("upper_body")
	rarm_waist = moveit_commander.MoveGroupCommander("rarm_with_waist")

	upper.set_pose_reference_frame("base_link")
	rarm_waist.set_pose_reference_frame("base_link")

	## reset pose
	joint_goal = upper.get_current_joint_values()
	for i in range(len(joint_goal)):
		joint_goal[i] = 0
	joint_goal[6] = -3
	joint_goal[16] = -3
	upper.go(joint_goal, wait=True)

	## pose goal with rarm
	rarm_waist.set_end_effector_link("r_eef_pick_link")
	pose_goal = PoseStamped()
	pose_goal.header.frame_id = "base_link"
	pose_goal.pose.orientation = euler_to_quaternion(0, 0, 0)
	pose_goal.pose.position.x = 0.8
	pose_goal.pose.position.y = -0.2
	pose_goal.pose.position.z = 1.3
	rarm_waist.set_pose_target(pose_goal)
	rarm_waist.go(wait=True)

	## cylinder add
	cylinder_name = "wine_bottle"
	cylinder_pose = PoseStamped()
	cylinder_pose.header.frame_id = "r_eef_pick_link"
	cylinder_pose.pose.position.x = 0.1
	cylinder_pose.pose.orientation = euler_to_quaternion(0, 0, 0)
	scene.add_cylinder(cylinder_name, cylinder_pose, .3, .05) # height, radius
	rospy.sleep(SLEEP_TIME)

	## cylinder attach
	grasping_group = 'rarm_with_waist'
	touch_links = robot.get_link_names(group=grasping_group)
	eef_link = rarm_waist.get_end_effector_link()
	scene.attach_box(eef_link, cylinder_name, touch_links=touch_links) # there is no attach_cylinder
	rospy.sleep(SLEEP_TIME)

	## pose goal with rarm
	## pour
	pose_goal.pose.orientation = euler_to_quaternion(-math.radians(80), 0, 0)
	pose_goal.pose.position.x = 0.6
	pose_goal.pose.position.y = -0.1
	pose_goal.pose.position.z = 1.2
	rarm_waist.set_pose_target(pose_goal)
	rarm_waist.go(wait=True)

	## pose goal with rarm
	## release
	pose_goal.pose.orientation = euler_to_quaternion(0, 0, 0)
	pose_goal.pose.position.x = 0.8
	pose_goal.pose.position.y = -0.2
	pose_goal.pose.position.z = 1.3
	rarm_waist.set_pose_target(pose_goal)
	rarm_waist.go(wait=True)

	## cylinder release
	scene.remove_attached_object(eef_link, name=cylinder_name)
	rospy.sleep(SLEEP_TIME)

	## reset pose
	joint_goal = upper.get_current_joint_values()
	for i in range(len(joint_goal)):
		joint_goal[i] = 0
	joint_goal[6] = -3
	joint_goal[16] = -3
	upper.go(joint_goal, wait=True)

	## cylinder remove
	scene.remove_world_object(cylinder_name)
	rospy.sleep(SLEEP_TIME)


def main():
	"""
	gazebo 空間の立方体に置かれたボトルの中身をコップに注ぐ動作をします
	"""
	rospy.init_node("navi")

	client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
	client.wait_for_server()

	for i, pose in enumerate(waypoints):
		goal = goal_pose(pose)
		client.send_goal(goal)
		client.wait_for_result()

		if i == 1:
			pour()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		exit()

#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

######################################
# MAIN
######################################

def main():

    # init node
    rospy.init_node("moveit_tutorial_node")

    print("ROS node is created")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

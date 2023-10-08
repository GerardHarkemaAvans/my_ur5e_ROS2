#! /usr/bin/env python

import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)

if 0:
	import rospy
	import sys
	import copy
	import moveit_msgs.msg
	import geometry_msgs.msg
	import moveit_commander
	from std_msgs.msg import Empty
	import math
	import actionlib

def main(args=None):
    rclpy.init(args=args)
    print('Hi from my_demo.')
    ###################################################################
    # MoveItPy Setup
    ###################################################################
    logger = get_logger("moveit_py.pose_goal")



    # instantiate MoveItPy instance and get planning component
    panda = MoveItPy(node_name="my_demo")
    
    if 0:

        moveit_commander.roscpp_initialize(sys.argv)
        robot=moveit_commander.RobotCommander()
        scene=moveit_commander.PlanningSceneInterface()
        group=moveit_commander.MoveGroupCommander('arm')
        display_trajectory_publisher=rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)

        print("== go to home ==")
        target_values= group.get_named_target_values("home")
        group.go(target_values, wait = True)

        print("== go to left ==")
        target_values= group.get_named_target_values("left")
        group.go(target_values, wait = True)

        print("== move down, 50 mm ==")
        pose=group.get_current_pose()
        posetarget = pose
        posetarget.pose.position.z-=0.05
        group.set_pose_target(posetarget)
        plan=group.plan()
        group.go(wait=True)

        print("== go to right ==")
        target_values= group.get_named_target_values("right")
        group.go(target_values, wait = True)

        print("== go to home ==")
        target_values= group.get_named_target_values("home")
        group.go(target_values, wait = True)

        print("== go to resting ==")
        target_values= group.get_named_target_values("resting")
        group.go(target_values, wait = True)

        print("== ready ==")

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

#initialise moveit_commander and a rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_inferace', anonymous=True)

#instantiate a RobotCommander object
robot = moveit_commander.RobotCommander()

#instantitate a PlanningSceneInterface object
scene = moveit_commander.PlanningSceneInterface()

#instantitate a MoveGroupCommander object
group_name="manipulator"
move_group=moveit_commander.MoveGroupCommander(group_name)

#Create DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

#Go to Home Position
home = [2.15339e-05, -2.26790, 2.24914, -1.62, -1.6, -0.33310]
move_group.set_joint_value_target(home)
move_group.go(wait=True)

rospy.sleep(1)

#Go to above source table
source = [-3.34562,-1.43934,1.50303,-1.62,-1.6,-0.33310]
move_group.set_joint_value_target(source)
move_group.go(source, wait=True)

rospy.sleep(1)

#Go to above object on source table
source_object = [-0.8616, 0.17825, 1.3065, -0.72637, -0.68707, -0.00936, 0.014766]
move_group.set_pose_target(source_object)
move_group.go(source_object, wait=True)

rospy.sleep(1)

#Move towards object in z-direction
pickpose = move_group.get_current_pose().pose
pickpose.position.z -= 0.5
move_group.set_pose_target(pickpose)
move_group.go(pickpose, wait=True)

rospy.sleep(1)

#Move upwards in z-direction
pickpose = move_group.get_current_pose().pose
pickpose.position.z += 0.5
move_group.set_pose_target(pickpose)
move_group.go(pickpose, wait=True)

rospy.sleep(1)

#Move above assembly table
assembly = [-1.53122, -1.22303, 1.22964, -1.62, -1.6, -0.33310]
move_group.set_joint_value_target(assembly)
move_group.go(assembly, wait=True)

rospy.sleep(1)

#Move above plate on assembly table
assembly_plate = [-1.78539, -1.29920, 1.31691, -1.62, -1.6, -0.33310]
move_group.set_pose_target(assembly_plate)
move_group.go(assembly_plate)

#Move towards object in z-direction
placepose = move_group.get_current_pose().pose
placepose.position.z -= 0.5
move_group.set_pose_target(placepose)
move_group.go(placepose, wait=True)

rospy.sleep(1)

#Move upwards in z-direction
placepose = move_group.get_current_pose().pose
placepose.position.z += 0.5
move_group.set_pose_target(placepose)
move_group.go(placepose, wait=True)

rospy.sleep(1)

#Return to Home Position
move_group.set_joint_value_target(home)
move_group.go(wait=True)

move_group.stop()
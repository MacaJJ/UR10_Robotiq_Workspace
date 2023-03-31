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
from tf.transformations import quaternion_from_euler

#initialise moveit_commander and a rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_inferace', anonymous=True)

#instantiate a RobotCommander object
robot = moveit_commander.RobotCommander()

#instantitate a PlanningSceneInterface object
scene = moveit_commander.PlanningSceneInterface()

#instantitate a MoveGroupCommander object
arm_group=moveit_commander.MoveGroupCommander("manipulator")
hand_group=moveit_commander.MoveGroupCommander("gripper")

#Create DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)


#Go to Home Position
arm_group.set_named_target("HomePosition")
arm_group.go(wait=True)

rospy.sleep(1)

#Go to above source table
source = [-3.34562,-1.43934, 1.50303,-1.57,-1.57,0]
arm_group.set_joint_value_target(source)
arm_group.go(wait=True)

rospy.sleep(1)

#Go to above object on source table (Replace with camera)
source_object = geometry_msgs.msg.Pose()

roll_angle = pi
pitch_angle = 0
yaw_angle = pi/2

quartenion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)
source_object.orientation.x = quartenion[0]
source_object.orientation.y = quartenion[1]
source_object.orientation.z = quartenion[2]
source_object.orientation.w = quartenion[3]
source_object.position.x = -0.86
source_object.position.y = 0.18
source_object.position.z = 1.31
arm_group.set_pose_target(source_object)
arm_group.go(wait=True)

rospy.sleep(1)

#Move towards object in z-direction
pickpose = arm_group.get_current_pose().pose
pickpose.position.z -= 0.41
arm_group.set_pose_target(pickpose)
arm_group.go(wait=True)

rospy.sleep(1)


half_closed = [0.36, 0.36, 0.36, 0.36, 0.36, 0.36]

hand_group.set_joint_value_target(half_closed)
hand_group.go()

rospy.sleep(1)

#Move upwards in z-direction
pickpose = arm_group.get_current_pose().pose
pickpose.position.z += 0.41
arm_group.set_pose_target(pickpose)
arm_group.go(wait=True)

rospy.sleep(1)

#Move above assembly table
assembly = [-1.53122, -1.22303, 1.22964, -1.57, -1.57, 0]
arm_group.set_joint_value_target(assembly)
arm_group.go(wait=True)

rospy.sleep(1)

#Move above plate on assembly table (Replace with camera)
assembly_plate = geometry_msgs.msg.Pose()
roll_angle = 0
pitch_angle = pi
yaw_angle = 0

quartenion = quaternion_from_euler(roll_angle, pitch_angle, yaw_angle)
assembly_plate.orientation.x = quartenion[0]
assembly_plate.orientation.y = quartenion[1]
assembly_plate.orientation.z = quartenion[2]
assembly_plate.orientation.w = quartenion[3]
assembly_plate.position.x = -0.02
assembly_plate.position.y = -0.870
assembly_plate.position.z = 1.32
arm_group.set_pose_target(assembly_plate)
arm_group.go(wait=True)

#Move towards object in z-direction
placepose = arm_group.get_current_pose().pose
placepose.position.z -= 0.4
arm_group.set_pose_target(placepose)
arm_group.go(wait=True)

rospy.sleep(1)

hand_group.set_named_target("Gripper_Open")
hand_group.go()

rospy.sleep(1)

#Move upwards in z-direction
placepose = arm_group.get_current_pose().pose
placepose.position.z += 0.4
arm_group.set_pose_target(placepose)
arm_group.go(wait=True)

rospy.sleep(1)

#Return to Home Position
arm_group.set_named_target("HomePosition")
arm_group.go(wait=True)

arm_group.stop()

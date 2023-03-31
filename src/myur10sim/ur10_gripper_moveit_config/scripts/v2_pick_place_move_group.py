#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
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

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

object_pose = None

def pos_callback(PoseStamped_data):
	global object_pose

	center_pose = PoseStamped()
	center_pose.header.stamp = PoseStamped_data.header.stamp
	# camera_pose.header.frame_id = "camera_link"
	center_pose.header.frame_id = PoseStamped_data.header.frame_id
	center_pose.pose.position.x = PoseStamped_data.pose.position.x
	center_pose.pose.position.y = PoseStamped_data.pose.position.y
	center_pose.pose.position.z = PoseStamped_data.pose.position.z

	center_pose.pose.orientation.x = PoseStamped_data.pose.orientation.x
	center_pose.pose.orientation.y = PoseStamped_data.pose.orientation.y
	center_pose.pose.orientation.z = PoseStamped_data.pose.orientation.z
	center_pose.pose.orientation.w = PoseStamped_data.pose.orientation.w

	center_to_camera_transform = TransformStamped()
	center_to_camera_transform.header.frame_id = "camera_depth_frame"
	center_to_camera_transform.child_frame_id = "camera_link"
	center_to_camera_transform.transform.translation.x = 0
	center_to_camera_transform.transform.translation.y = 0
	center_to_camera_transform.transform.translation.z = 0

	center_to_camera_quaternion = quaternion_from_euler(-pi/2, 0,-pi/2)
	center_to_camera_transform.transform.rotation.x = center_to_camera_quaternion[0]
	center_to_camera_transform.transform.rotation.y = center_to_camera_quaternion[1]
	center_to_camera_transform.transform.rotation.z = center_to_camera_quaternion[2]
	center_to_camera_transform.transform.rotation.w = center_to_camera_quaternion[3]	


	# camera_to_wrist3_transform = TransformStamped()
	# camera_to_wrist3_transform.header.frame_id = "camera_link"
	# camera_to_wrist3_transform.child_frame_id = "wrist_3_link"
	# camera_to_wrist3_transform.transform.translation.x = 0
	# camera_to_wrist3_transform.transform.translation.y = -0.1
	# camera_to_wrist3_transform.transform.translation.z = 0

	# camera_to_wrist3_quarternion = quaternion_from_euler(0, 0, 0)
	# camera_to_wrist3_transform.transform.rotation.x = camera_to_wrist3_quarternion[0]
	# camera_to_wrist3_transform.transform.rotation.y = camera_to_wrist3_quarternion[1]
	# camera_to_wrist3_transform.transform.rotation.z = camera_to_wrist3_quarternion[2]
	# camera_to_wrist3_transform.transform.rotation.w = camera_to_wrist3_quarternion[3]

	try:
		# wrist3_to_robot_transform_stamped = tf_buffer.lookup_transform("base_link", "wrist_3_link", rospy.Time())
		# object_to_wrist3_pose = tf2_geometry_msgs.do_transform_pose(camera_pose, camera_to_wrist3_transform)
		# object_pose = tf2_geometry_msgs.do_transform_pose(object_to_wrist3_pose, wrist3_to_robot_transform_stamped)
		
		camera_to_robot_transform_stamped = tf_buffer.lookup_transform("base_link", "camera_link", rospy.Time())
		center_to_camera_transform_stamped = tf2_geometry_msgs.do_transform_pose(center_pose, center_to_camera_transform)

		object_pose = tf2_geometry_msgs.do_transform_pose(center_to_camera_transform_stamped, camera_to_robot_transform_stamped)
		# object_pose = tf2_geometry_msgs.do_transform_pose(center_pose, camera_to_robot_transform_stamped)

		# print("Received object pose wrt to robot base", object_pose)
		# print(camera_to_robot_transform_stamped)
	except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		rospy.logerr("Failed to transform object pose from camera frame to robot's frame")

rospy.Subscriber('/red_object/position', PoseStamped, pos_callback)

#Go to Home Position
print(">> Going to Home Position")
arm_group.set_named_target("HomePosition")
arm_group.go(wait=True)

rospy.sleep(1)

#Go to above source table
source = [-3.34562,-pi/2, pi/2,-pi/2 ,-pi/2, 0]

print(">> Going above source table")
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
source_object.position.x = object_pose.pose.position.x
source_object.position.y = object_pose.pose.position.y
source_object.position.z = 1.34

print(">> Going above detected object")
print(source_object)
arm_group.set_pose_target(source_object)
arm_group.go(wait=True)

rospy.sleep(1)

print(">> Making final adjustments")
source_object.position.x = object_pose.pose.position.x
source_object.position.y = object_pose.pose.position.y
arm_group.set_pose_target(source_object)
arm_group.go(wait=True)

rospy.sleep(1)

#Move towards object in z-direction
print(">> Moving 0.45 meters downwards along the z-axis")
pickpose = arm_group.get_current_pose().pose
pickpose.position.z -= 0.45
arm_group.set_pose_target(pickpose)
arm_group.go(wait=True)

rospy.sleep(1)


half_closed = [0.36, 0.36, 0.36, 0.36, 0.36, 0.36]

print(">> Closing gripper")
hand_group.set_joint_value_target(half_closed)
hand_group.go()

rospy.sleep(1)

#Move upwards in z-direction
print(">> Moving 0.45 meters upwards along the z-axis")
pickpose = arm_group.get_current_pose().pose
pickpose.position.z += 0.45
arm_group.set_pose_target(pickpose)
arm_group.go(wait=True)

rospy.sleep(1)

#Move above assembly table
print(">> Going above assembly table")
assembly = [-1.53122, -pi/2, pi/2, -pi/2, -pi/2, 0]
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
assembly_plate.position.x = object_pose.pose.position.x
assembly_plate.position.y = object_pose.pose.position.y
assembly_plate.position.z = 1.34

print(">> Going above detected plate on assembly table")
arm_group.set_pose_target(assembly_plate)
print(assembly_plate)
arm_group.go(wait=True)

rospy.sleep(1)

print(">> Making final adjustments")
assembly_plate.position.x= object_pose.pose.position.x
assembly_plate.position.y= object_pose.pose.position.y
print(assembly_plate)

arm_group.set_pose_target(assembly_plate)
arm_group.go(wait=True)

#Move towards object in z-direction
print(">> Moving 0.43 meters downpwards along the z-axis")
placepose = arm_group.get_current_pose().pose
placepose.position.z -= 0.43
arm_group.set_pose_target(placepose)
arm_group.go(wait=True)

rospy.sleep(1)

print(">> Opening gripper")
hand_group.set_named_target("Gripper_Open")
hand_group.go()

rospy.sleep(1)

#Move upwards in z-direction
print(">> Moving 0.43 meters upwards along the z-axis")
placepose = arm_group.get_current_pose().pose
placepose.position.z += 0.43
arm_group.set_pose_target(placepose)
arm_group.go(wait=True)

rospy.sleep(1)

#Return to Home Position
print(">> Returning to Home Position")
arm_group.set_named_target("HomePosition")
arm_group.go(wait=True)

arm_group.stop()

#!/usr/bin/env python3
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf2_ros
import tf2_geometry_msgs
import math
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from detection_msgs.msg import Detection2DArray, Detection2D
from moveit_commander.conversions import pose_to_list
from tf.transformations import quaternion_from_euler

class TransformPose:
	def __init__(self):
		#initialise moveit_commander and a rospy node
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('PickPlace_Node', anonymous=True)

		#instantiate a RobotCommander object
		robot = moveit_commander.RobotCommander()

		#instantitate a PlanningSceneInterface object
		scene = moveit_commander.PlanningSceneInterface()

		#instantitate a MoveGroupCommander object
		self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
		self.hand_group = moveit_commander.MoveGroupCommander("gripper")

		self.arm_group.set_num_planning_attempts(10)

		#Create DisplayTrajectory ROS publisher which is used to display trajectories in Rviz
		display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

		self.tf_buffer = tf2_ros.Buffer()
		tf_listener = tf2_ros.TransformListener(self.tf_buffer)

		#Subscribe to object pose
		self.object_pose = rospy.Subscriber('/yolov5/detections', Detection2DArray, self.detection_callback)

		#Subscribe to plate pose
		self.plate_pose_sub = rospy.Subscriber('/red_object/position', PoseStamped, self.pos_callback)

	def pos_callback(self, PoseStamped_data):

		plate_pose = PoseStamped()
		plate_pose.header.stamp = PoseStamped_data.header.stamp
		# camera_pose.header.frame_id = "camera_link"
		plate_pose.header.frame_id = PoseStamped_data.header.frame_id
		plate_pose.pose.position.x = PoseStamped_data.pose.position.x
		plate_pose.pose.position.y = PoseStamped_data.pose.position.y
		plate_pose.pose.position.z = PoseStamped_data.pose.position.z

		plate_pose.pose.orientation.x = PoseStamped_data.pose.orientation.x
		plate_pose.pose.orientation.y = PoseStamped_data.pose.orientation.y
		plate_pose.pose.orientation.z = PoseStamped_data.pose.orientation.z
		plate_pose.pose.orientation.w = PoseStamped_data.pose.orientation.w

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

		try:
			camera_to_robot_transform_stamped = self.tf_buffer.lookup_transform("base_link", "camera_link", rospy.Time())
			center_to_camera_transform_stamped = tf2_geometry_msgs.do_transform_pose(plate_pose, center_to_camera_transform)

			self.plate_pose = tf2_geometry_msgs.do_transform_pose(center_to_camera_transform_stamped, camera_to_robot_transform_stamped)

			# print("Received object pose wrt to robot base", plate_pose)
			# print(camera_to_robot_transform_stamped)
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			rospy.logerr("Failed to transform object pose from camera frame to robot's frame")

	def detection_callback(self, Detection2DArray_data):
		
		for detection in Detection2DArray_data.detections:

			detected_obj = Detection2D()
			detected_obj.header.stamp = detection.header.stamp
			detected_obj.header.frame_id = detection.header.frame_id
			
			detected_obj.bbox = detection.bbox

			detected_obj.results.Class = detection.results.Class
			detected_obj.results.score = detection.results.score

			detected_obj.results.pose = detection.results.pose

			detected_pose_stamped = PoseStamped()
			detected_pose_stamped.header.stamp = detected_obj.header.stamp
			detected_pose_stamped.header.frame_id = detected_obj.header.frame_id

			detected_pose_stamped.pose = detected_obj.results.pose
			
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

			try:
				camera_to_robot_transform_stamped = self.tf_buffer.lookup_transform("base_link", "camera_link", rospy.Time())
				center_to_camera_transform_stamped = tf2_geometry_msgs.do_transform_pose(detected_pose_stamped, center_to_camera_transform)

				self.object_pose = tf2_geometry_msgs.do_transform_pose(center_to_camera_transform_stamped, camera_to_robot_transform_stamped)

				# print("Received object pose wrt to robot base", plate_pose)
				# print(camera_to_robot_transform_stamped)
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				rospy.logerr("Failed to transform object pose from camera frame to robot's frame")


class PickPlace:
	def __init__(self):
		self.tp = TransformPose()

	def HomePosition(self):
		print(">> Going to Home Position")
		self.tp.arm_group.set_named_target("HomePosition")
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def SourceTable(self):
		#Go to above source table
		source = [-3.34562,-pi/2, pi/2,-pi/2 ,-pi/2, -pi/16]

		print(">> Going above source table")
		self.tp.arm_group.set_joint_value_target(source)
		self.tp.arm_group.go(wait=True)

		print(">> Sleeping for 20 seconds for YOLO to catch up")
		rospy.sleep(20)

	def OrientationConstraint(self):
		orientation_constraint = moveit_msgs.msg.OrientationConstraint()
		orientation_constraint.header.frame_id = "base_link"
		orientation_constraint.link_name = "wrist_3_link"
		orientation_constraint.orientation = geometry_msgs.msg.Quaternion(w=1, x=0, y=0, z=0)
		orientation_constraint.absolute_x_axis_tolerance = 0.1
		orientation_constraint.absolute_y_axis_tolerance = 0.1
		orientation_constraint.absolute_z_axis_tolerance = 0.05
		orientation_constraint.weight = 1.0

		path_constraints = moveit_msgs.msg.Constraints()
		path_constraints.orientation_constraints.append(orientation_constraint)
		self.tp.arm_group.set_path_constraints(path_constraints)


	def SourceTable_Object(self):

		#Go to above object on source table (Replace with camera)
		source_object = geometry_msgs.msg.Pose()

		# quartenion = quaternion_from_euler(pi, 0, pi/2)
		# source_object.orientation.x = quartenion[0]
		# source_object.orientation.y = quartenion[1]
		# source_object.orientation.z = quartenion[2]
		# source_object.orientation.w = quartenion[3]
		
		source_object.position.x = self.tp.object_pose.pose.position.x
		source_object.position.y = self.tp.object_pose.pose.position.y
		source_object.position.z = 1.34

		# magnitude = math.sqrt(self.tp.object_pose.pose.orientation.x**2 + self.tp.object_pose.pose.orientation.y **2 
		# 					+ self.tp.object_pose.pose.orientation.z**2 + self.tp.object_pose.pose.orientation.w **2)

		# print(magnitude)
		# print(self.tp.object_pose.pose.orientation)

		source_object.orientation.x = (self.tp.object_pose.pose.orientation.x)
		source_object.orientation.y = (self.tp.object_pose.pose.orientation.y)
		source_object.orientation.z = (self.tp.object_pose.pose.orientation.z)
		source_object.orientation.w = (self.tp.object_pose.pose.orientation.w)

		# print(self.tp.object_pose.pose.orientation)
		# print(source_object.orientation)

		rospy.sleep(1)

		print(">> Going above detected object")
		print(source_object)
		# self.tp.arm_group.set_start_state_to_current_state()
		# self.tp.arm_group.set_pose_target(source_object)
		# plan = self.tp.arm_group.plan()
		# self.tp.arm_group.execute(plan)
		self.tp.arm_group.set_pose_target(source_object)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

		# rospy.sleep(20)

		# print(">> Making final adjustments")
		# print(source_object)
		# source_object.position.x = self.tp.object_pose.pose.position.x
		# source_object.position.y = self.tp.object_pose.pose.position.y
		# source_object.orientation.x = (self.tp.object_pose.pose.orientation.x)
		# source_object.orientation.y = (self.tp.object_pose.pose.orientation.y)
		# source_object.orientation.z = (self.tp.object_pose.pose.orientation.z)
		# source_object.orientation.w = (self.tp.object_pose.pose.orientation.w)
		# self.tp.arm_group.set_pose_target(source_object)
		# self.tp.arm_group.go(wait=True)

		# rospy.sleep(1)

	def Pick(self):

		#Move towards object in z-direction
		print(">> Moving 0.45 meters downwards along the z-axis")
		pickpose = self.tp.arm_group.get_current_pose().pose
		pickpose.position.z -= 0.45
		self.tp.arm_group.set_pose_target(pickpose)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

		half_closed = [0.36, 0.36, 0.36, 0.36, 0.36, 0.36]

		print(">> Closing gripper")
		self.tp.hand_group.set_joint_value_target(half_closed)
		self.tp.hand_group.go()

		rospy.sleep(1)

		#Move upwards in z-direction
		print(">> Moving 0.45 meters upwards along the z-axis")
		pickpose = self.tp.arm_group.get_current_pose().pose
		pickpose.position.z += 0.45

		self.tp.arm_group.set_pose_target(pickpose)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def AssemblyTable(self):
		#Move above assembly table
		print(">> Going above assembly table")
		assembly = [-1.53122, -pi/2, pi/2, -pi/2, -pi/2, pi/2]
		self.tp.arm_group.set_joint_value_target(assembly)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def AssemblyTable_Object(self):
		#Move above plate on assembly table (Replace with camera)
		assembly_plate = geometry_msgs.msg.Pose()

		quartenion = quaternion_from_euler(0, pi, 0)
		assembly_plate.orientation.x = quartenion[0]
		assembly_plate.orientation.y = quartenion[1]
		assembly_plate.orientation.z = quartenion[2]
		assembly_plate.orientation.w = quartenion[3]
		
		assembly_plate.position.x = self.tp.plate_pose.pose.position.x
		assembly_plate.position.y = self.tp.plate_pose.pose.position.y
		assembly_plate.position.z = 1.34

		print(">> Going above detected plate on assembly table")
		self.tp.arm_group.set_pose_target(assembly_plate)
		print(assembly_plate)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

		print(">> Making final adjustments")
		assembly_plate.position.x= self.tp.plate_pose.pose.position.x
		assembly_plate.position.y= self.tp.plate_pose.pose.position.y
		print(assembly_plate)

		self.tp.arm_group.set_pose_target(assembly_plate)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def Place(self):
		#Move towards object in z-direction
		print(">> Moving 0.43 meters downpwards along the z-axis")
		placepose = self.tp.arm_group.get_current_pose().pose
		placepose.position.z -= 0.43
		self.tp.arm_group.set_pose_target(placepose)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

		print(">> Opening gripper")
		self.tp.hand_group.set_named_target("Gripper_Open")
		self.tp.hand_group.go()

		rospy.sleep(1)

		#Move upwards in z-direction
		print(">> Moving 0.43 meters upwards along the z-axis")
		placepose = self.tp.arm_group.get_current_pose().pose
		placepose.position.z += 0.43
		self.tp.arm_group.set_pose_target(placepose)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

		self.tp.arm_group.clear_path_constraints()

if __name__ == '__main__':
	pickplace_object = PickPlace()
	# pickplace_object.HomePosition()
	pickplace_object.SourceTable()
	# pickplace_object.OrientationConstraint()
	pickplace_object.SourceTable_Object()
	pickplace_object.Pick()
	pickplace_object.AssemblyTable()
	pickplace_object.AssemblyTable_Object()
	pickplace_object.Place()
	pickplace_object.HomePosition()

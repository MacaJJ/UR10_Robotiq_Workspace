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
import numpy as np
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

		#Subscribe to red plate pose
		self.red_plate_pose_sub = rospy.Subscriber('/pose/red', Detection2DArray, self.red_pos_callback)

		#Subscribe to blue plate pose
		self.blue_plate_pose_sub = rospy.Subscriber('/pose/blue', Detection2DArray, self.blue_pos_callback)

	def red_pos_callback(self, plate_data):

		red_plate_pose_list = []

		for detection in plate_data.detections:

			plate_pose = Detection2D()
			plate_pose.header.stamp = detection.header.stamp
			plate_pose.header.frame_id = detection.header.frame_id

			#Unused in the code
			plate_pose.bbox = detection.bbox 
			plate_pose.results.score = detection.results.score

			plate_pose.results.Class = detection.results.Class
			# plate_pose.results.pose = detection.results.pose

			plate_pose_stamped = PoseStamped()
			plate_pose_stamped.header.stamp = plate_pose.header.stamp
			plate_pose_stamped.header.frame_id = plate_pose.header.frame_id

			plate_pose_stamped.pose = detection.results.pose

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
				center_to_camera_transform_stamped = tf2_geometry_msgs.do_transform_pose(plate_pose_stamped, center_to_camera_transform)

				plate_pose_base = tf2_geometry_msgs.do_transform_pose(center_to_camera_transform_stamped, camera_to_robot_transform_stamped)

				plate_pose.results.pose = plate_pose_base

				red_plate_pose_list.append(plate_pose)

				# print("Received object pose wrt to robot base", plate_pose)
				# print(camera_to_robot_transform_stamped)
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				rospy.logerr("Failed to transform object pose from camera frame to robot's frame")

		self.red_plate_pose_list = red_plate_pose_list

	def blue_pos_callback(self, plate_data):

		blue_plate_pose_list = []

		for detection in plate_data.detections:

			plate_pose = Detection2D()
			plate_pose.header.stamp = detection.header.stamp
			plate_pose.header.frame_id = detection.header.frame_id

			#Unused in the code
			plate_pose.bbox = detection.bbox 
			plate_pose.results.score = detection.results.score

			plate_pose.results.Class = detection.results.Class
			# plate_pose.results.pose = detection.results.pose

			plate_pose_stamped = PoseStamped()
			plate_pose_stamped.header.stamp = plate_pose.header.stamp
			plate_pose_stamped.header.frame_id = plate_pose.header.frame_id

			plate_pose_stamped.pose = detection.results.pose

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
				center_to_camera_transform_stamped = tf2_geometry_msgs.do_transform_pose(plate_pose_stamped, center_to_camera_transform)

				plate_pose_base = tf2_geometry_msgs.do_transform_pose(center_to_camera_transform_stamped, camera_to_robot_transform_stamped)

				plate_pose.results.pose = plate_pose_base

				blue_plate_pose_list.append(plate_pose)

				# print("Received object pose wrt to robot base", plate_pose)
				# print(camera_to_robot_transform_stamped)
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				rospy.logerr("Failed to transform object pose from camera frame to robot's frame")

		self.blue_plate_pose_list = blue_plate_pose_list

	def detection_callback(self, Detection2DArray_data):

		green_plate_pose_list = []
		
		for detection in Detection2DArray_data.detections:

			detected_obj = Detection2D()
			detected_obj.header.stamp = detection.header.stamp
			detected_obj.header.frame_id = detection.header.frame_id
			
			detected_obj.bbox = detection.bbox

			detected_obj.results.Class = detection.results.Class
			detected_obj.results.score = detection.results.score

			# detected_obj.results.pose = detection.results.pose

			detected_pose_stamped = PoseStamped()
			detected_pose_stamped.header.stamp = detected_obj.header.stamp
			detected_pose_stamped.header.frame_id = detected_obj.header.frame_id

			# detected_pose_stamped.pose = detected_obj.results.pose
			detected_pose_stamped.pose = detection.results.pose
			
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

				object_pose = tf2_geometry_msgs.do_transform_pose(center_to_camera_transform_stamped, camera_to_robot_transform_stamped)

				detected_obj.results.pose = object_pose

				if detected_obj.results.Class == "green_cube":
					green_plate_pose_list.append(detected_obj)
				else:
					self.detected_obj = detected_obj

				# print("Received object pose wrt to robot base", plate_pose)
				# print(camera_to_robot_transform_stamped)
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
				rospy.logerr("Failed to transform object pose from camera frame to robot's frame")

		self.green_plate_pose_list = green_plate_pose_list


class PickPlace:
	def __init__(self):
		self.tp = TransformPose()

	def HomePosition(self):
		print(">> Going to Home Position")
		self.tp.arm_group.set_named_target("HomePosition")
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def AssemblyTable(self):
		#Move above assembly table
		print(">> Going above assembly table")
		assembly = [-1.53122, -pi/2, pi/2, -pi/2, -pi/2, 0]
		self.tp.arm_group.set_joint_value_target(assembly)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def Mean_XYZ(self, pose_list):
		pose_results = [pose.results.pose for pose in pose_list]

		x_list = [pose.pose.position.x for pose in pose_results]
		y_list = [pose.pose.position.y for pose in pose_results]
		z_list = [pose.pose.position.z for pose in pose_results]

		x_mean = np.mean(x_list)
		y_mean = np.mean(y_list)
		z_mean = np.mean(z_list)

		mean_pose = np.array([x_mean, y_mean, z_mean])

		return mean_pose


	def SortContainers(self):

		red_plate_container_pose_list = []
		blue_plate_container_pose_list = []

		red_plate_container_pose_list = self.tp.red_plate_pose_list
		blue_plate_container_pose_list = self.tp.blue_plate_pose_list

		red_average = self.Mean_XYZ(self.tp.red_plate_pose_list)
		blue_average = self.Mean_XYZ(self.tp.blue_plate_pose_list)

		# print(self.tp.green_plate_pose_list)

		for green_plate in self.tp.green_plate_pose_list:
			green_x = green_plate.results.pose.pose.position.x
			green_y = green_plate.results.pose.pose.position.y
			green_z = green_plate.results.pose.pose.position.z

			greencube_pos = np.array([green_x, green_y, green_z])

			red_distance = np.linalg.norm(greencube_pos - red_average)
			blue_distance = np.linalg.norm(greencube_pos - blue_average)

			if red_distance < blue_distance:
				red_plate_container_pose_list.append(green_plate)
			else:
				blue_plate_container_pose_list.append(green_plate)

		self.red_plate_container_pose_list = red_plate_container_pose_list
		self.blue_plate_container_pose_list = blue_plate_container_pose_list

		print(">> Poses on Red Tray")
		print(self.red_plate_container_pose_list)
		print(">> Poses on Blue Tray")
		print(self.blue_plate_container_pose_list)

	def AssemblyTable_SortPosition(self):

		print(">> Sleeping for 16 seconds for YOLO to catch up")
		rospy.sleep(16)

		self.SortContainers()

		red_container_visit_count = {}
		blue_container_visit_count = {}

		for pose in self.red_plate_container_pose_list:
			x_pos = pose.results.pose.pose.position.x
			y_pos = pose.results.pose.pose.position.y
			z_pos = pose.results.pose.pose.position.z

			if pose.results.Class == 'green_cube':
				red_container_visit_count[(x_pos, y_pos, z_pos)] = 0

		for pose in self.blue_plate_container_pose_list:
			x_pos = pose.results.pose.pose.position.x
			y_pos = pose.results.pose.pose.position.y
			z_pos = pose.results.pose.pose.position.z

			if pose.results.Class == 'green_cube':
				blue_container_visit_count[(x_pos, y_pos, z_pos)] = 0

		# for pose in self.tp.plate_pose_list:
		# 	x_pos = pose.results.pose.pose.position.x
		# 	y_pos = pose.results.pose.pose.position.y
		# 	z_pos = pose.results.pose.pose.position.z

		# 	if pose.results.score == 1:
		# 		red_plate_list.append(pose)

		# 		#There are 3 containers, so we need to keep track which container has been visited already
		# 		if pose.results.Class == 'container':
		# 			red_container_visit_count[(x_pos, y_pos, z_pos)] = 0


		# 		if pose.results.Class == 'plate':
		# 			red_plate_visit_count[(x_pos, y_pos, z_pos)] = 0
		# 		elif pose.results.Class == 'bowl':
		# 			red_bowl_visit_count = {(x_pos, y_pos, z_pos): 0}
		# 		elif pose.results.Class == 'cup':
		# 			red_cup_visit_count = {(x_pos, y_pos, z_pos): 0}
		# 		elif pose.results.Class == 'container':
		# 			red_container_visit_count = {(x_pos, y_pos, z_pos): 0}
		# 		else:
		# 			print("Unknown contour detected in saved plate pose list")

		# 	elif pose.results.score == 0.5:
		# 		blue_plate_list.append(pose)

		# 		if pose.results.Class == 'container':
		# 			blue_container_visit_count[(x_pos, y_pos, z_pos)] = 0

		# 	else:
		# 		print("Pose belong to neither yellow or blue. Pose will be discarded")

		self.red_container_visit_count = red_container_visit_count
		self.blue_container_visit_count = blue_container_visit_count

	def SourceTable(self):
		#Go to above source table
		source = [-3.34562,-pi/2, pi/2,-pi/2 ,-pi/2, -pi/16]

		print(">> Going above source table")
		self.tp.arm_group.set_joint_value_target(source)
		self.tp.arm_group.go(wait=True)

		print(">> Sleeping for 16 seconds for YOLO to catch up")
		rospy.sleep(16)

	def MoveToPose(self, pose):
		self.tp.arm_group.set_pose_target(pose)
		self.tp.arm_group.go(wait=True)

		rospy.sleep(1)

	def SourceTable_Object(self, obj):

		#Go to above object on source table (Replace with camera)
		source_object = geometry_msgs.msg.Pose()
		
		source_object.position.x = self.tp.detected_obj.results.pose.pose.position.x
		source_object.position.y = self.tp.detected_obj.results.pose.pose.position.y
		source_object.position.z = 1.34

		self.source_object_depth = self.tp.detected_obj.results.pose.pose.position.z

		source_object.orientation.x = (self.tp.detected_obj.results.pose.pose.orientation.x)
		source_object.orientation.y = (self.tp.detected_obj.results.pose.pose.orientation.y)
		source_object.orientation.z = (self.tp.detected_obj.results.pose.pose.orientation.z)
		source_object.orientation.w = (self.tp.detected_obj.results.pose.pose.orientation.w)

		rospy.sleep(1)

		#Always printing blue_cube
		print(">> Going above detected object:", obj)
		print(source_object)
		
		self.MoveToPose(source_object)

		rospy.sleep(1)


	def Pick(self, depth):

		#Move towards object in z-direction
		pickpose = self.tp.arm_group.get_current_pose().pose

		print(">> Moving downwards along the z-axis")
						
		pickpose.position.z = depth + 0.7 + 0.12 #Adjust according to the height
		
		self.MoveToPose(pickpose)

		rospy.sleep(1)

		half_closed = [0.36, 0.36, 0.36, 0.36, 0.36, 0.36]

		print(">> Closing gripper")
		self.tp.hand_group.set_joint_value_target(half_closed)
		self.tp.hand_group.go()

		rospy.sleep(1)

		#Move upwards in z-direction
		print(">> Moving upwards along the z-axis")
		pickpose = self.tp.arm_group.get_current_pose().pose
		pickpose.position.z = 1.34

		self.MoveToPose(pickpose)


	def Place(self):
		#Move towards object in z-direction
		print(">> Moving 0.30 meters downpwards along the z-axis")
		placepose = self.tp.arm_group.get_current_pose().pose
		placepose.position.z -= 0.30
		
		self.MoveToPose(placepose)

		print(">> Opening gripper")
		self.tp.hand_group.set_named_target("Gripper_Open")
		self.tp.hand_group.go()

		rospy.sleep(1)

		#Move upwards in z-direction
		print(">> Moving 0.30 meters upwards along the z-axis")
		placepose = self.tp.arm_group.get_current_pose().pose
		placepose.position.z += 0.30
		
		self.MoveToPose(placepose)

		rospy.sleep(1)

	def PnP_Location1(self, plate_list, container_visit_count):

		pick_count_1 = {'red_cube': 2, 'blue_cube': 1}
		place_count_1 = {'red_cube': 0, 'blue_cube': 0}

		print(plate_list)

		self.SourceTable()
	
		while any(place_count_1[obj] < pick_count_1[obj] for obj in pick_count_1):
			for obj in pick_count_1:
				while place_count_1[obj] < pick_count_1[obj]:
					if self.tp.detected_obj.results.Class == obj:
						if obj == 'red_cube':

							self.SourceTable_Object(obj)
							
							self.Pick(self.source_object_depth) 

							#So that the robot arm won't take a longer path
							self.AssemblyTable()
							
							for pose in plate_list:
								x_pos = pose.results.pose.pose.position.x
								y_pos = pose.results.pose.pose.position.y
								z_pos = pose.results.pose.pose.position.z

								if pose.results.Class == 'plate':

									plate_arm_pose = geometry_msgs.msg.Pose()

									plate_arm_pose.position.x = x_pos
									plate_arm_pose.position.y = y_pos
									plate_arm_pose.position.z = 1.34

									quartenion = quaternion_from_euler(0, pi, 0)
									plate_arm_pose.orientation.x = quartenion[0]
									plate_arm_pose.orientation.y = quartenion[1]
									plate_arm_pose.orientation.z = quartenion[2]
									plate_arm_pose.orientation.w = quartenion[3]

									break

							print(">>Going above plate")
							print(plate_arm_pose)


						elif obj == 'blue_cube':

							self.SourceTable_Object(obj)
							
							self.Pick(self.source_object_depth) 

							#So that the robot arm won't take a longer path
							self.AssemblyTable()
							
							for pose in plate_list:
								x_pos = pose.results.pose.pose.position.x
								y_pos = pose.results.pose.pose.position.y
								z_pos = pose.results.pose.pose.position.z

								if pose.results.Class == 'green_cube' and container_visit_count[(x_pos, y_pos, z_pos)] == 0:

									plate_arm_pose = geometry_msgs.msg.Pose()

									plate_arm_pose.position.x = x_pos
									plate_arm_pose.position.y = y_pos
									plate_arm_pose.position.z = 1.34

									quartenion = quaternion_from_euler(0, pi, 0)
									plate_arm_pose.orientation.x = quartenion[0]
									plate_arm_pose.orientation.y = quartenion[1]
									plate_arm_pose.orientation.z = quartenion[2]
									plate_arm_pose.orientation.w = quartenion[3]

									container_visit_count[(x_pos, y_pos, z_pos)] += 1

									break

							print(">>Going above container")
							print(plate_arm_pose)

						self.MoveToPose(plate_arm_pose)
						
						self.Place()

						place_count_1[obj] += 1

						rospy.sleep(1)

						self.SourceTable()
					
				else:
					print("All ", obj, "has been placed")

		print(">> All objects from Location 1 have been placed")

	# def PnP_Location2(self, plate_list):
	# 	pick_count_2 = {}
	# 	place_count_2 = {}

	# 	self.SourceTable() #Change to MoveTo location

	# 	while any(place_count_2[obj] < pick_count_2[obj] for obj in pick_count_2):
	# 		for obj in pick_count_2:
	# 			while place_count_2[obj] < pick_count_2[obj]:
	# 				if self.tp.detected_obj.results.Class == obj:
	# 					if obj == 'red_cube':

	# 						self.SourceTable_Object()
							
	# 						self.Pick(self.source_object_depth) 

	# 						#So that the robot arm won't take a longer path
	# 						self.AssemblyTable()
								
	# 						for pose in plate_list:
	# 							x_pos = pose.results.pose.pose.position.x
	# 							y_pos = pose.results.pose.pose.position.y
	# 							z_pos = pose.results.pose.pose.position.z

	# 							if pose.results.Class == 'plate':

	# 								plate_arm_pose = geometry_msgs.msg.Pose()

	# 								plate_arm_pose.position.x = x_pos
	# 								plate_arm_pose.position.y = y_pos
	# 								plate_arm_pose.position.z = 1.34

	# 								quartenion = quaternion_from_euler(0, pi, 0)
	# 								plate_arm_pose.orientation.x = quartenion[0]
	# 								plate_arm_pose.orientation.y = quartenion[1]
	# 								plate_arm_pose.orientation.z = quartenion[2]
	# 								plate_arm_pose.orientation.w = quartenion[3]

	# 								break

	# 						print(">>Going above ", pose.results.Class)
	# 						print(plate_arm_pose)

	# 						self.MoveToPose(plate_arm_pose)
							
	# 						self.Place()

	# 						place_count_2[obj] += 1

	# 						rospy.sleep(1)

	# 						self.SourceTable()
						
	# 				else:
	# 					print("No class match found")

	# 		print(">> All objects from Location 2 have been placed")

	# def PnP_Location3(self, plate_list):

	# 	pick_count_3 = {}
	# 	place_count_3 = {}

	# 	self.SourceTable() #Change to MoveTo location

	# 	while any(place_count_3[obj] < pick_count_3[obj] for obj in pick_count_3):
	# 		for obj in pick_count_3:
	# 			while place_count_3[obj] < pick_count_3[obj]:
	# 				if self.tp.detected_obj.results.Class == obj:
	# 					if obj == 'red_cube':

	# 						self.SourceTable_Object()
							
	# 						self.Pick(self.source_object_depth) 

	# 						#So that the robot arm won't take a longer path
	# 						self.AssemblyTable()
								
	# 						for pose in plate_list:
	# 							x_pos = pose.results.pose.pose.position.x
	# 							y_pos = pose.results.pose.pose.position.y
	# 							z_pos = pose.results.pose.pose.position.z

	# 							if pose.results.Class == 'plate':

	# 								plate_arm_pose = geometry_msgs.msg.Pose()

	# 								plate_arm_pose.position.x = x_pos
	# 								plate_arm_pose.position.y = y_pos
	# 								plate_arm_pose.position.z = 1.34

	# 								quartenion = quaternion_from_euler(0, pi, 0)
	# 								plate_arm_pose.orientation.x = quartenion[0]
	# 								plate_arm_pose.orientation.y = quartenion[1]
	# 								plate_arm_pose.orientation.z = quartenion[2]
	# 								plate_arm_pose.orientation.w = quartenion[3]

	# 								break

	# 						print(">>Going above ", pose.results.Class)
	# 						print(plate_arm_pose)

	# 						self.MoveToPose(plate_arm_pose)
							
	# 						self.Place()

	# 						place_count_3[obj] += 1

	# 						rospy.sleep(1)

	# 						self.SourceTable()
						
	# 				else:
	# 					print("No class match found")

	# 		print(">> All objects from Location 3 have been placed")


if __name__ == '__main__':
	pickplace_object = PickPlace()
	
	pickplace_object.HomePosition()

	pickplace_object.AssemblyTable()
	pickplace_object.AssemblyTable_SortPosition()
	
	pickplace_object.PnP_Location1(pickplace_object.red_plate_container_pose_list, pickplace_object.red_container_visit_count)
	# pickplace_object.PnP_Location2(pickplace_object.tp.red_plate_pose_list)
	# pickplace_object.PnP_Location3(pickplace_object.tp.red_plate_pose_list)

	pickplace_object.PnP_Location1(pickplace_object.blue_plate_container_pose_list, pickplace_object.blue_container_visit_count)
	# pickplace_object.PnP_Location2(pickplace_object.tp.blue_plate_pose_list)
	# pickplace_object.PnP_Location3(pickplace_object.tp.blue_plate_pose_list)

	pickplace_object.HomePosition()

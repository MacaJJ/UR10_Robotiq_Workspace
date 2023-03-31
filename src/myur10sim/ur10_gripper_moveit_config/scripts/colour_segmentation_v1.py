#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_from_euler
from math import pi

class DepthCamera:
	def __init__(self):
		rospy.init_node('ObjectDetector')

		# Initialize CvBridge
		self.bridge = CvBridge()

		# Subscriber for the color image
		self.color_sub = rospy.Subscriber('/myur10/camera1/color/image_raw', Image, self.color_callback)

		# Subscriber for the depth image
		self.depth_sub = rospy.Subscriber('/myur10/camera1/depth/image_raw', Image, self.depth_callback)

		# Publisher for output image
		self.image_pub = rospy.Publisher('/output_image', Image, queue_size=1)

		# Publisher for object pose
		self.object_pub = rospy.Publisher('/red_object/position', PoseStamped, queue_size=1)

		# Subscriber for depth camera's intrinsic parameters
		camera_info = rospy.wait_for_message('/myur10/camera1/depth/camera_info', CameraInfo)
		self.fx = camera_info.K[0]
		self.fy = camera_info.K[4]
		self.pp_cx = camera_info.K[2]
		self.pp_cy = camera_info.K[5]

	def color_callback(self, color_data):
		try:
			color_image = self.bridge.imgmsg_to_cv2(color_data, "bgr8")
		except CvBridgeError as e:
			print(e)

		lower_red = np.array([0, 0, 60], dtype="uint8")
		upper_red = np.array([100, 50, 255], dtype="uint8")
		mask = cv2.inRange(color_image, lower_red, upper_red)
		output = cv2.bitwise_and(color_image, color_image, mask=mask)

		kernel = np.ones((3,3),np.uint8)
		erosion = cv2.erode(output, kernel, iterations = 4)
		dilate = cv2.dilate(erosion, kernel)

		gray = cv2.cvtColor(dilate, cv2.COLOR_BGR2GRAY)

		self.gray = gray

		contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

		for cnt in contours:
			cv2.drawContours(color_image, [cnt], -1, (255,0,0), 3)
			M = cv2.moments(cnt)
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
			cv2.circle(color_image,(cx,cy), 5, (255,0,255), -1)

		self.image_pub.publish(self.bridge.cv2_to_imgmsg(color_image, "bgr8"))

	def depth_callback(self, depth_data):
		try:
			depth_image = self.bridge.imgmsg_to_cv2(depth_data, "32FC1")
		except CvBridgeError as e:
			print(e)

		if self.gray.any():
			contours, hierarchy = cv2.findContours(self.gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

			for cnt in contours:
				M = cv2.moments(cnt)
				cx = int(M['m10']/M['m00'])
				cy = int(M['m01']/M['m00'])

				object_pose = PoseStamped()
				object_pose.header.stamp = rospy.Time.now()
				object_pose.header.frame_id = "camera_depth_frame"
			
				depth = depth_image[cy, cx]

				x_meters = (cx - self.pp_cx) * depth / self.fx
				y_meters = (cy - self.pp_cy) * depth / self.fy
				z_meters = depth

				object_pose.pose.position.x = x_meters
				object_pose.pose.position.y = y_meters
				object_pose.pose.position.z = z_meters

				object_pose_quaternion = quaternion_from_euler(0, 0, 0)
				object_pose.pose.orientation.x = object_pose_quaternion[0]
				object_pose.pose.orientation.y = object_pose_quaternion[1]
				object_pose.pose.orientation.z = object_pose_quaternion[2]
				object_pose.pose.orientation.w = object_pose_quaternion[3]

				self.object_pub.publish(object_pose)

				# print(object_pose)
				
if __name__ == '__main__':
	try:
		depth_camera = DepthCamera()
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting down the ROS ObjectDetector Node')
		cv2.destroyAllWindows()


# def image_callback(msg):
# 	bridge = CvBridge()

# 	try:
# 		image = bridge.imgmsg_to_cv2(msg, "bgr8")
# 	except CvBridgeError as e:
# 		print(e)

# 	#Get camera intrinsic parameters
# 	camera_info = rospy.wait_for_message('/myur10/camera1/depth/camera_info', CameraInfo)
# 	fx = camera_info.K[0]
# 	fy = camera_info.K[4]
# 	pp_cx = camera_info.K[2]
# 	pp_cy = camera_info.K[5]

# 	# img_width = camera_info.width
# 	# img_height = camera_info.height

# 	# #Convert pixels to meters
# 	# sensor_width = img_width*fx
# 	# horizontal_fov = 2*np.arctan2(sensor_width, 2*fx)
# 	# x_conversion = 1/ (2*np.tan(horizontal_fov/2))

# 	# sensor_height = img_height*fy
# 	# vertical_fov = 2*np.arctan2(sensor_height, 2*fy)
# 	# y_conversion = 0.75*1/ (2*np.tan(vertical_fov/2))

# 	rows, cols = depth_image.shape
# 	u, v = np.meshgrid(np.arange(cols), np.arange(rows), indexing='xy')

# 	x = (u - pp_cx) * depth_image/fx
# 	y = (v - pp_cy) * depth_image/fy

# 	lower_red = np.array([0, 0, 60], dtype="uint8")
# 	upper_red = np.array([100, 50, 255], dtype="uint8")
# 	mask = cv2.inRange(image, lower_red, upper_red)
# 	output = cv2.bitwise_and(image, image, mask=mask)

# 	kernel = np.ones((3,3),np.uint8)
# 	erosion = cv2.erode(output, kernel, iterations = 4)
# 	dilate = cv2.dilate(erosion, kernel)

# 	gray = cv2.cvtColor(dilate, cv2.COLOR_BGR2GRAY)

# 	contours, hierarchy = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
# 	contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

# 	for cnt in contours:
# 		cv2.drawContours(image, [cnt], -1, (255,0,0), 3)
# 		M = cv2.moments(cnt)
# 		cx = int(M['m10']/M['m00'])
# 		cy = int(M['m01']/M['m00'])
# 		cv2.circle(image,(cx,cy), 5, (255,0,255), -1)
# 		x_meters = x[cy][cx]
# 		y_meters = y[cy][cx]

# 		#Object Pose with respect to the center of image
# 		object_pose = Pose()
# 		object_pose.position.x = x_meters
# 		object_pose.position.y = y_meters
# 		object_pose.position.z = depth_image[cy][cx]/1000
		
# 		object_pose_quaternion = quaternion_from_euler(0, 0, 0)
# 		object_pose.orientation.x = object_pose_quaternion[0]
# 		object_pose.orientation.y = object_pose_quaternion[1]
# 		object_pose.orientation.z = object_pose_quaternion[2]
# 		object_pose.orientation.w = object_pose_quaternion[3]

# 		object_pub.publish(object_pose)


# 	cv2.imshow("Result", image)
# 	# cv2.imshow("Mask", output)
# 	cv2.waitKey(1)

# rospy.init_node('Red_Object_Detector')
# rospy.Subscriber('/myur10/camera1/depth/image_raw', Image, image_callback)
# object_pub = rospy.Publisher('red_object/position', Pose, queue_size=1)
# rospy.spin()
# cv2.destroyAllWindows()

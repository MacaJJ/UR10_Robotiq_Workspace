#!/usr/bin/env python3

#Python Node for reading camera data

import sys, time
import numpy as np
import cv2
import roslib
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class image_read:
	def __init__(self):
		self.bridge = CvBridge()

		#Define the subscribed topic
		self.subscriber = rospy.Subscriber("/output_image", Image, self.callback, queue_size=1)

	def callback(self, ros_data):
		#Images are read and processed here

		try:
			image = self.bridge.imgmsg_to_cv2(ros_data, "bgr8")
		except CvBridgeError as e:
			print(e)

		cv2.namedWindow('Output_Image', cv2.WINDOW_NORMAL)
		cv2.imshow('Output_Image', image)
		cv2.waitKey(2)

def main(args):
	#Initialises and cleanup ros node
	ic = image_read()
	rospy.init_node('image_read')
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print('Shutting down the ROS Image Reader Node')
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
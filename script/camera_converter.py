#!/usr/bin/env python

################################################################################
## {Description}: Subscribing (original size image ) and Publish (resized)
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

"""
Image published (CompressedImage) from tello originally size of 960x720 pixels
We will try to resize it using imutils.resize (with aspect ratio) to width = 320
and then republish it as Image
"""

# import the necessary Python packages
from __future__ import print_function
import sys
import cv2
import time
import numpy as np
import imutils

# import the necessary ROS packages
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, Image, CameraInfo

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

import rospy

class CameraConverter:
	def __init__(self):

		self.bridge = CvBridge()
		self.cvtCameraInfo = CameraInfo()

		rospy.logwarn("Camera Converter Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Subscribe to CompressedImage msg
		self.image_topic = "/tello/image_raw/compressed"
		self.image_sub = rospy.Subscriber(
						self.image_topic, 
						CompressedImage, 
						self.cbImage
						)

		# Publish to Image msg
		self.cvImage_topic = "/tello/image_raw_resized"
		self.cvImage_pub = rospy.Publisher(
						self.cvImage_topic, 
						Image, 
						queue_size=10
						)

		# Publish to CameraInfo msg
		self.cameraInfo_topic = "/tello/camerainfo_resized"
		self.cameraInfo_pub = rospy.Publisher(
						self.cameraInfo_topic, 
						CameraInfo, 
						queue_size=10
						)

		# Allow up to one second to connection
		rospy.sleep(1)

	# Convert image to OpenCV format
	def cbImage(self, msg):

		try:
			# direct conversion to cv2
			self.cv_image = self.bridge.compressed_imgmsg_to_cv2(
								msg, 
								"bgr8"
								)

			# resize the image
			self.cv_image = imutils.resize(
						self.cv_image, 
						height=500
						)
		except CvBridgeError as e:
			print(e)

		try:
			# publish the resized image
			self.cvImage_pub.publish(
					self.bridge.cv2_to_imgmsg(
							self.cv_image, 
							"bgr8"
							)
						)

		except CvBridgeError as e:
			print(e)		

		try:
			# publish the resized image
			self.cvtCameraInfo.height = self.cv_image.shape[0]
			self.cvtCameraInfo.width = self.cv_image.shape[1]

			self.cameraInfo_pub.publish(
						self.cvtCameraInfo
						)

		except CvBridgeError as e:
			print(e)

	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("Camera Converter Node [OFFLINE]...")

if __name__ == '__main__':

	# Initialize
	rospy.init_node('camera_converter', anonymous=False)
	camera = CameraConverter()

	# Loops
	while not rospy.is_shutdown():
		try:
			rospy.spin()
		except KeyboardInterrupt:
			camera.cbShutdown()

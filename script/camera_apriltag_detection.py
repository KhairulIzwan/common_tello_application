#!/usr/bin/env python

################################################################################
## {Description}: Detecting an Apriltag3
## {Description}: Publish /isApriltag topic
## {Description}: If AprilTag3 detected; /isApriltag --> True
## {Description}: If AprilTag3 detected; /isApriltag --> False
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
import random
import apriltag

# import the necessary ROS packages
from std_msgs.msg import String, Bool, Float32
from sensor_msgs.msg import Image, CompressedImage

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

from tello_driver.msg import TelloStatus

from nav_msgs.msg import Odometry

from sensor_msgs.msg import Imu

from common_tello_application.msg import apriltagN as apriltagList
from common_tello_application.msg import apriltagC as apriltagCenter
from common_tello_application.msg import apriltagH as apriltagHomography
from common_tello_application.msg import apriltagCorner

import rospy

class CameraAprilTag:
	def __init__(self):

		# OpenCV -- ROS
		self.bridge = CvBridge()

		# AprilTag3 
		self.detector = apriltag.Detector()

		self.isApriltag = Bool()
		self.isApriltagN = apriltagList()
		self.apriltagCenterX = apriltagCenter()
		self.apriltagCenterY = apriltagCenter()
		self.apriltagH00 = apriltagHomography()
		self.apriltagH01 = apriltagHomography()
		self.apriltagH02 = apriltagHomography()
		self.apriltagH10 = apriltagHomography()
		self.apriltagH11 = apriltagHomography()
		self.apriltagH12 = apriltagHomography()
		self.apriltagH20 = apriltagHomography()
		self.apriltagH21 = apriltagHomography()
		self.apriltagH22 = apriltagHomography()
		self.apriltagCornerX1 = apriltagCorner()
		self.apriltagCornerY1 = apriltagCorner()
		self.apriltagCornerX2 = apriltagCorner()
		self.apriltagCornerY2 = apriltagCorner()
		self.apriltagCornerX3 = apriltagCorner()
		self.apriltagCornerY3 = apriltagCorner()
		self.apriltagCornerX4 = apriltagCorner()
		self.apriltagCornerY4 = apriltagCorner()

		# state
		self.image_received = False
		self.isApriltagDistance_received = False

		rospy.logwarn("AprilTag Detection Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

		# Subscribe to CompressedImage msg
		self.telloImage_topic = "/tello/image_raw/compressed"
		self.telloImage_sub = rospy.Subscriber(
						self.telloImage_topic, 
						CompressedImage, 
						self.cbImage
						)

		# Publish to Bool msg
		self.isApriltag_topic = "/isApriltag"
		self.isApriltag_pub = rospy.Publisher(
					self.isApriltag_topic, 
					Bool, 
					queue_size=10
					)

		# Publish to apriltagList msg
		self.isApriltagN_topic = "/isApriltag/N"
		self.isApriltagN_pub = rospy.Publisher(
					self.isApriltagN_topic, 
					apriltagList, 
					queue_size=10
					)

		# Subscribe to TelloStatus msg
		self.telloStatus_topic = "/tello/status"
		self.telloStatus_sub = rospy.Subscriber(
				self.telloStatus_topic, 
				TelloStatus, 
				self.cbTelloStatus
				)

		# Subscribe to Odometry msg
		self.telloOdom_topic = "/tello/odom"
		self.telloOdom_sub = rospy.Subscriber(
				self.telloOdom_topic, 
				Odometry, 
				self.cbTelloOdometry
				)

		# Subscribe to PoseWithCovariance msg
		self.telloIMU_topic = "/tello/imu"
		self.telloIMU_sub = rospy.Subscriber(
				self.telloIMU_topic, Imu, 
				self.cbTelloIMU
				)

		# Subscribe to Float32 msg
		self.apriltagDistance_topic = "/isApriltag/Corner/Distance"
		self.apriltagDistance_sub = rospy.Subscriber(
					self.apriltagDistance_topic, 
					Float32, 
					self.cbIsApriltagDistance
					)

		# Publish to apriltagCenter msg
		self.apriltagCenterX_topic = "/isApriltag/Center/X"
		self.apriltagCenterX_pub = rospy.Publisher(
					self.apriltagCenterX_topic, 
					apriltagCenter, 
					queue_size=10
					)

		# Publish to apriltagCenter msg
		self.apriltagCenterY_topic = "/isApriltag/Center/Y"
		self.apriltagCenterY_pub = rospy.Publisher(
					self.apriltagCenterY_topic, 
					apriltagCenter, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H00_topic = "/isApriltag/Homography/H00"
		self.apriltagHomography_H00_pub = rospy.Publisher(
					self.apriltagHomography_H00_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H01_topic = "/isApriltag/Homography/H01"
		self.apriltagHomography_H01_pub = rospy.Publisher(
					self.apriltagHomography_H01_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H02_topic = "/isApriltag/Homography/H02"
		self.apriltagHomography_H02_pub = rospy.Publisher(
					self.apriltagHomography_H02_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H10_topic = "/isApriltag/Homography/H10"
		self.apriltagHomography_H10_pub = rospy.Publisher(
					self.apriltagHomography_H10_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H11_topic = "/isApriltag/Homography/H11"
		self.apriltagHomography_H11_pub = rospy.Publisher(
					self.apriltagHomography_H11_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H12_topic = "/isApriltag/Homography/H12"
		self.apriltagHomography_H12_pub = rospy.Publisher(
					self.apriltagHomography_H12_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H20_topic = "/isApriltag/Homography/H20"
		self.apriltagHomography_H20_pub = rospy.Publisher(
					self.apriltagHomography_H20_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H21_topic = "/isApriltag/Homography/H21"
		self.apriltagHomography_H21_pub = rospy.Publisher(
					self.apriltagHomography_H21_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagHomography msg
		self.apriltagHomography_H22_topic = "/isApriltag/Homography/H22"
		self.apriltagHomography_H22_pub = rospy.Publisher(
					self.apriltagHomography_H22_topic, 
					apriltagHomography, 
					queue_size=10
					)

		# Publish to apriltagCorner msg
		self.apriltagCorner_X1_topic = "/isApriltag/Corner/X1"
		self.apriltagCorner_X1_pub = rospy.Publisher(
					self.apriltagCorner_X1_topic, 
					apriltagCorner, 
					queue_size=10
					)

		# Publish to apriltagCorner msg
		self.apriltagCorner_Y1_topic = "/isApriltag/Corner/Y1"
		self.apriltagCorner_Y1_pub = rospy.Publisher(
					self.apriltagCorner_Y1_topic, 
					apriltagCorner, 
					queue_size=10
					)

		# Publish to apriltagCorner msg
		self.apriltagCorner_X2_topic = "/isApriltag/Corner/X2"
		self.apriltagCorner_X2_pub = rospy.Publisher(
					self.apriltagCorner_X2_topic, 
					apriltagCorner, 
					queue_size=10
					)

		# Publish to apriltagCorner msg
		self.apriltagCorner_Y2_topic = "/isApriltag/Corner/Y2"
		self.apriltagCorner_Y2_pub = rospy.Publisher(
					self.apriltagCorner_Y2_topic, 
					apriltagCorner, 
					queue_size=10
					)

		# Publish to apriltagCorner msg
		self.apriltagCorner_X3_topic = "/isApriltag/Corner/X3"
		self.apriltagCorner_X3_pub = rospy.Publisher(
					self.apriltagCorner_X3_topic, 
					apriltagCorner, 
					queue_size=10
					)

		# Publish to apriltagCorner msg
		self.apriltagCorner_Y3_topic = "/isApriltag/Corner/Y3"
		self.apriltagCorner_Y3_pub = rospy.Publisher(
					self.apriltagCorner_Y3_topic, 
					apriltagCorner, 
					queue_size=10
					)

		# Publish to apriltagCorner msg
		self.apriltagCorner_X4_topic = "/isApriltag/Corner/X4"
		self.apriltagCorner_X4_pub = rospy.Publisher(
					self.apriltagCorner_X4_topic, 
					apriltagCorner, 
					queue_size=10
					)

		# Publish to apriltagCorner msg
		self.apriltagCorner_Y4_topic = "/isApriltag/Corner/Y4"
		self.apriltagCorner_Y4_pub = rospy.Publisher(
					self.apriltagCorner_Y4_topic, 
					apriltagCorner, 
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
		except CvBridgeError as e:
			print(e)

		if self.cv_image is not None:
			self.image_received = True
		else:
			self.image_received = False

	def cbIsApriltagDistance(self, msg):
		try:
			self.isApriltagDistance = msg.data
		except AttributeError as e:
			print(e)

		if self.isApriltagDistance is not None:
			self.isApriltagDistance_received = True
		else:
			self.isApriltagDistance_received = False

	# Get TelloIMU info
	def cbTelloIMU(self, msg):
	
		self.orientationXIMU = msg.orientation.x
		self.orientationYIMU = msg.orientation.y
		self.orientationZIMU = msg.orientation.z
		self.orientationWIMU = msg.orientation.w
		
		self.angularXIMU = msg.angular_velocity.x
		self.angularYIMU = msg.angular_velocity.y
		self.angularZIMU = msg.angular_velocity.z
		
		self.linearXIMU = msg.linear_acceleration.x
		self.linearYIMU = msg.linear_acceleration.y
		self.linearZIMU = msg.linear_acceleration.z
		
	# Get TelloOdometry info
	def cbTelloOdometry(self, msg):
	
		self.poseX = msg.pose.pose.position.x
		self.poseY = msg.pose.pose.position.y
		self.poseZ = msg.pose.pose.position.z
		self.orientationX = msg.pose.pose.orientation.x
		self.orientationY = msg.pose.pose.orientation.y
		self.orientationZ = msg.pose.pose.orientation.z
		self.orientationW = msg.pose.pose.orientation.w
		
		self.linearX = msg.twist.twist.linear.x
		self.linearY = msg.twist.twist.linear.y
		self.linearZ = msg.twist.twist.linear.z
		self.angularX = msg.twist.twist.angular.x
		self.angularY = msg.twist.twist.angular.y
		self.angularZ = msg.twist.twist.angular.z

	# Get TelloStatus info
	def cbTelloStatus(self, msg):

		# Non-negative; calibrated to takeoff altitude; auto-calib if 
		# falls below takeoff height; inaccurate near ground
		self.height_m = msg.height_m

		self.speed_northing_mps = msg.speed_northing_mps
		self.speed_easting_mps = msg.speed_easting_mps
		self.speed_horizontal_mps = msg.speed_horizontal_mps
		self.speed_vertical_mps = msg.speed_vertical_mps

		self.flight_time_sec = msg.flight_time_sec

		self.imu_state = msg.imu_state
		self.pressure_state = msg.pressure_state
		self.down_visual_state = msg.down_visual_state
		self.power_state = msg.power_state
		self.battery_state = msg.battery_state
		self.gravity_state = msg.gravity_state
		self.wind_state = msg.wind_state

		self.imu_calibration_state = msg.imu_calibration_state
		self.battery_percentage = msg.battery_percentage
		self.drone_fly_time_left_sec = msg.drone_fly_time_left_sec
		self.drone_battery_left_sec = msg.drone_battery_left_sec

		self.is_flying = msg.is_flying
		self.is_on_ground = msg.is_on_ground
		# is_em_open True in flight, False when landed
		self.is_em_open = msg.is_em_open
		self.is_drone_hover = msg.is_drone_hover
		self.is_outage_recording = msg.is_outage_recording
		self.is_battery_low = msg.is_battery_low
		self.is_battery_lower = msg.is_battery_lower
		self.is_factory_mode = msg.is_factory_mode

		# flymode=1: landed; =6: flying
		self.fly_mode = msg.fly_mode
		self.throw_takeoff_timer_sec = msg.throw_takeoff_timer_sec
		self.camera_state = msg.camera_state

		self.electrical_machinery_state = msg.electrical_machinery_state

		self.front_in = msg.front_in
		self.front_out = msg.front_out
		self.front_lsc = msg.front_lsc

		self.temperature_height_m = msg.temperature_height_m

		self.cmd_roll_ratio = msg.cmd_roll_ratio
		self.cmd_pitch_ratio = msg.cmd_pitch_ratio
		self.cmd_yaw_ratio = msg.cmd_yaw_ratio
		self.cmd_vspeed_ratio = msg.cmd_vspeed_ratio
		self.cmd_fast_mode = msg.cmd_fast_mode

	# Image information callback
	def cbInfo(self):

		fontFace = cv2.FONT_HERSHEY_PLAIN
		fontScale = 0.7
		color = (255, 255, 255)
		colorPose = (0, 0, 255)
		colorIMU = (255, 0, 255)
		thickness = 1
		lineType = cv2.LINE_AA
		bottomLeftOrigin = False # if True (text upside down)

		# Status
		cv2.putText(self.cv_image, 
			"height_m: %.4f" % (self.height_m), 
			(10, 10), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"speed_northing_mps: %.4f" % (self.speed_northing_mps), 
			(10, 30), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"speed_easting_mps: %.4f" % (self.speed_easting_mps), 
			(10, 40), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"speed_horizontal_mps: %.4f" % (self.speed_horizontal_mps), 
			(10, 50), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"speed_vertical_mps: %.4f" % (self.speed_vertical_mps), 
			(10, 60), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"flight_time_sec: %.4f" % (self.flight_time_sec), 
			(10, 80), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"imu_state: %s" % (self.imu_state), 
			(10, 100), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)			

		cv2.putText(self.cv_image, 
			"pressure_state: %s" % (self.pressure_state), 
			(10, 110), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)						

		cv2.putText(self.cv_image, 
			"down_visual_state: %s" % (self.down_visual_state), 
			(10, 120), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"pressure_state: %s" % (self.pressure_state), 
			(10, 130), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"power_state: %s" % (self.power_state), 
			(10, 140), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"battery_state: %s" % (self.battery_state), 
			(10, 150), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"gravity_state: %s" % (self.gravity_state), 
			(10, 160), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"wind_state: %s" % (self.wind_state), 
			(10, 170), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"imu_calibration_state: %d" % (self.imu_calibration_state), 
			(10, 190), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"battery_percentage: %d" % (self.battery_percentage), 
			(10, 200), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"drone_fly_time_left_sec: %.4f" % (self.drone_fly_time_left_sec), 
			(10, 210), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"drone_battery_left_sec: %.4f" % (self.drone_battery_left_sec), 
			(10, 220), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"is_flying: %s" % (self.is_flying), 
			(10, 240), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"is_on_ground: %s" % (self.is_on_ground), 
			(10, 250), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"is_em_open: %s" % (self.is_em_open), 
			(10, 260), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"is_drone_hover: %s" % (self.is_drone_hover), 
			(10, 270), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"is_outage_recording: %s" % (self.is_outage_recording), 
			(10, 280), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"is_battery_low: %s" % (self.is_battery_low), 
			(10, 290), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"is_battery_lower: %s" % (self.is_battery_lower), 
			(10, 300), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"is_factory_mode: %s" % (self.is_factory_mode), 
			(10, 310), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"fly_mode: %d" % (self.fly_mode), 
			(10, 330), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"throw_takeoff_timer_sec: %.4f" % (self.throw_takeoff_timer_sec), 
			(10, 340), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"camera_state: %d" % (self.camera_state), 
			(10, 350), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"electrical_machinery_state: %d" % (self.electrical_machinery_state), 
			(10, 370), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"front_in: %s" % (self.front_in), 
			(10, 390), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"front_out: %s" % (self.front_out), 
			(10, 400), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"front_lsc: %s" % (self.front_lsc), 
			(10, 410), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"temperature_height_m: %.4f" % (self.temperature_height_m), 
			(10, 430), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"cmd_roll_ratio: %.4f" % (self.cmd_roll_ratio), 
			(10, 450), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"cmd_pitch_ratio: %.4f" % (self.cmd_pitch_ratio), 
			(10, 460), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"cmd_yaw_ratio: %.4f" % (self.cmd_yaw_ratio), 
			(10, 470), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"cmd_vspeed_ratio: %.4f" % (self.cmd_vspeed_ratio), 
			(10, 480), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"cmd_fast_mode: %d" % (self.cmd_fast_mode), 
			(10, 490), 
			fontFace, 
			fontScale, 
			color, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		# ODOM
		cv2.putText(self.cv_image, 
			"ODOMPose: X:%.4f, Y:%.4f, Y:%.4f" % (self.poseX, self.poseY, self.poseZ), 
			(10, 510), 
			fontFace, 
			fontScale, 
			colorPose, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"ODOMOrientation: X:%.4f, Y:%.4f, Z:%.4f, W:%.4f" % (self.orientationX, self.orientationY, self.orientationZ, self.orientationW), 
			(10, 520), 
			fontFace, 
			fontScale, 
			colorPose, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"ODOMvelLinear: X:%.4f, Y:%.4f, Z:%.4f" % (self.linearX, self.linearY, self.linearZ), 
			(10, 530), 
			fontFace, 
			fontScale, 
			colorPose, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"ODOMvelAngular: X:%.4f, Y:%.4f, Z:%.4f" % (self.angularX, self.angularY, self.angularZ), 
			(10, 540), 
			fontFace, 
			fontScale, 
			colorPose, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		# IMU
		cv2.putText(self.cv_image, 
			"IMUOrientation: X:%.4f, Y:%.4f, Z:%.4f, W:%.4f" % (self.orientationXIMU, self.orientationYIMU, self.orientationZIMU, self.orientationWIMU), 
			(10, 560), 
			fontFace, 
			fontScale, 
			colorIMU, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"IMUvelAngular: X:%.4f, Y:%.4f, Z:%.4f" % (self.angularXIMU, self.angularYIMU, self.angularZIMU), 
			(10, 570), 
			fontFace, 
			fontScale, 
			colorIMU, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(self.cv_image, 
			"IMUvelLinear: X:%.4f, Y:%.4f, Z:%.4f" % (self.linearXIMU, self.linearYIMU, self.linearZIMU), 
			(10, 580), 
			fontFace, 
			fontScale, 
			colorIMU, 
			thickness, 
			lineType, 
			bottomLeftOrigin)

	def cbAprilTag(self):
		# Info parameters configuration
		fontFace = cv2.FONT_HERSHEY_PLAIN
		fontScale = 0.7
		color = (255, 255, 255)
		colorPose = (0, 0, 255)
		colorIMU = (255, 0, 255)
		thickness = 1
		lineType = cv2.LINE_AA
		bottomLeftOrigin = False # if True (text upside down)

		# Converting to grayscale
		cv_image_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)

		# AprilTag detection
		result = self.detector.detect(cv_image_gray)

		# Is any Apriltag detected?
		if len(result) != 0:
			self.isApriltag.data = True
			self.isApriltag_pub.publish(self.isApriltag)

			self.apriltagN_list = []

			self.apriltagCenterX_list = []
			self.apriltagCenterY_list = []

			self.apriltagHomography_H00_list = []
			self.apriltagHomography_H01_list = []
			self.apriltagHomography_H02_list = []
			self.apriltagHomography_H10_list = []
			self.apriltagHomography_H11_list = []
			self.apriltagHomography_H12_list = []
			self.apriltagHomography_H20_list = []
			self.apriltagHomography_H21_list = []
			self.apriltagHomography_H22_list = []

			self.apriltagCorner_X1_list = []
			self.apriltagCorner_Y1_list = []
			self.apriltagCorner_X2_list = []
			self.apriltagCorner_Y2_list = []
			self.apriltagCorner_X3_list = []
			self.apriltagCorner_Y3_list = []
			self.apriltagCorner_X4_list = []
			self.apriltagCorner_Y4_list = []

			for i in range(len(result)):
				cv2.putText(
					self.cv_image, 
					"%d" % (result[i][1]), 
					(int(result[i][6][0]), int(result[i][6][1])), 
					fontFace, 
					fontScale * 5, 
					color, 
					thickness * 5, 
					lineType, 
					bottomLeftOrigin)

				cv2.line(
					self.cv_image, 
					(int(result[i][7][0][0]), int(result[i][7][0][1])), 
					(int(result[i][7][1][0]), int(result[i][7][1][1])), 
					(0, 0, 255), 
					5)

				cv2.line(
					self.cv_image, 
					(int(result[i][7][0][0]), int(result[i][7][0][1])), 
					(int(result[i][7][3][0]), int(result[i][7][3][1])), 
					(0, 255, 0), 
					5)

				cv2.line(
					self.cv_image, 
					(int(result[i][7][1][0]), int(result[i][7][1][1])), 
					(int(result[i][7][2][0]), int(result[i][7][2][1])), 
					(255, 0, 0), 
					5)

				cv2.line(
					self.cv_image, 
					(int(result[i][7][2][0]), int(result[i][7][2][1])), 
					(int(result[i][7][3][0]), int(result[i][7][3][1])), 
					(255, 0, 0), 
					5)

				cv2.circle(
					self.cv_image, 
					(int(result[i][6][0]), int(result[i][6][1])), 
					5, 
					(255, 0, 0), 
					-1)

				self.apriltagN_list.append(result[i][1])

				self.apriltagCenterX_list.append(result[i][6][0])
				self.apriltagCenterY_list.append(result[i][6][1])

				self.apriltagHomography_H00_list.append(result[i][5][0][0])
				self.apriltagHomography_H01_list.append(result[i][5][0][1])
				self.apriltagHomography_H02_list.append(result[i][5][0][2])
				self.apriltagHomography_H10_list.append(result[i][5][1][0])
				self.apriltagHomography_H11_list.append(result[i][5][1][1])
				self.apriltagHomography_H12_list.append(result[i][5][1][2])
				self.apriltagHomography_H20_list.append(result[i][5][2][0])
				self.apriltagHomography_H21_list.append(result[i][5][2][1])
				self.apriltagHomography_H22_list.append(result[i][5][2][2])

				self.apriltagCorner_X1_list.append(result[i][7][0][0])
				self.apriltagCorner_Y1_list.append(result[i][7][0][1])
				self.apriltagCorner_X2_list.append(result[i][7][1][0])
				self.apriltagCorner_Y2_list.append(result[i][7][1][1])
				self.apriltagCorner_X3_list.append(result[i][7][2][0])
				self.apriltagCorner_Y3_list.append(result[i][7][2][1])
				self.apriltagCorner_X4_list.append(result[i][7][3][0])
				self.apriltagCorner_Y4_list.append(result[i][7][3][1])

			self.isApriltagN.apriltagN = self.apriltagN_list
			self.isApriltagN_pub.publish(self.isApriltagN)

			self.apriltagCenterX.apriltagC = self.apriltagCenterX_list
			self.apriltagCenterY.apriltagC = self.apriltagCenterY_list
			self.apriltagCenterX_pub.publish(self.apriltagCenterX)
			self.apriltagCenterY_pub.publish(self.apriltagCenterY)

			self.apriltagH00.apriltagH = self.apriltagHomography_H00_list
			self.apriltagH01.apriltagH = self.apriltagHomography_H01_list
			self.apriltagH02.apriltagH = self.apriltagHomography_H02_list
			self.apriltagH10.apriltagH = self.apriltagHomography_H10_list
			self.apriltagH11.apriltagH = self.apriltagHomography_H11_list
			self.apriltagH12.apriltagH = self.apriltagHomography_H12_list
			self.apriltagH20.apriltagH = self.apriltagHomography_H20_list
			self.apriltagH21.apriltagH = self.apriltagHomography_H21_list
			self.apriltagH22.apriltagH = self.apriltagHomography_H22_list

			self.apriltagHomography_H00_pub.publish(self.apriltagH00)
			self.apriltagHomography_H01_pub.publish(self.apriltagH01)
			self.apriltagHomography_H02_pub.publish(self.apriltagH02)
			self.apriltagHomography_H10_pub.publish(self.apriltagH10)
			self.apriltagHomography_H11_pub.publish(self.apriltagH11)
			self.apriltagHomography_H12_pub.publish(self.apriltagH12)
			self.apriltagHomography_H20_pub.publish(self.apriltagH20)
			self.apriltagHomography_H21_pub.publish(self.apriltagH21)
			self.apriltagHomography_H22_pub.publish(self.apriltagH22)

			self.apriltagCornerX1.apriltagCorner = self.apriltagCorner_X1_list
			self.apriltagCornerY1.apriltagCorner = self.apriltagCorner_Y1_list
			self.apriltagCornerX2.apriltagCorner = self.apriltagCorner_X2_list
			self.apriltagCornerY2.apriltagCorner = self.apriltagCorner_Y2_list
			self.apriltagCornerX3.apriltagCorner = self.apriltagCorner_X3_list
			self.apriltagCornerY3.apriltagCorner = self.apriltagCorner_Y3_list
			self.apriltagCornerX4.apriltagCorner = self.apriltagCorner_X4_list
			self.apriltagCornerY4.apriltagCorner = self.apriltagCorner_Y4_list

			self.apriltagCorner_X1_pub.publish(self.apriltagCornerX1)
			self.apriltagCorner_Y1_pub.publish(self.apriltagCornerY1)
			self.apriltagCorner_X2_pub.publish(self.apriltagCornerX2)
			self.apriltagCorner_Y2_pub.publish(self.apriltagCornerY2)
			self.apriltagCorner_X3_pub.publish(self.apriltagCornerX3)
			self.apriltagCorner_Y3_pub.publish(self.apriltagCornerY3)
			self.apriltagCorner_X4_pub.publish(self.apriltagCornerX4)
			self.apriltagCorner_Y4_pub.publish(self.apriltagCornerY4)
		else:
			# AprilTag Detected?
			self.isApriltag.data = False
			self.isApriltag_pub.publish(self.isApriltag)

			# What AprilTag Detected?
			self.apriltagN_list = []
			self.isApriltagN.apriltagN = self.apriltagN_list
			self.isApriltagN_pub.publish(self.isApriltagN)

			self.apriltagCenterX_list = []
			self.apriltagCenterY_list = []
			self.apriltagCenterX.apriltagC = self.apriltagCenterX_list
			self.apriltagCenterY.apriltagC = self.apriltagCenterY_list
			self.apriltagCenterX_pub.publish(self.apriltagCenterX)
			self.apriltagCenterX_pub.publish(self.apriltagCenterX)

			self.apriltagHomography_H00_list = []
			self.apriltagHomography_H01_list = []
			self.apriltagHomography_H02_list = []
			self.apriltagHomography_H10_list = []
			self.apriltagHomography_H11_list = []
			self.apriltagHomography_H12_list = []
			self.apriltagHomography_H20_list = []
			self.apriltagHomography_H21_list = []
			self.apriltagHomography_H22_list = []

			self.apriltagH00.apriltagH = self.apriltagHomography_H00_list
			self.apriltagH01.apriltagH = self.apriltagHomography_H01_list
			self.apriltagH02.apriltagH = self.apriltagHomography_H02_list
			self.apriltagH10.apriltagH = self.apriltagHomography_H10_list
			self.apriltagH11.apriltagH = self.apriltagHomography_H11_list
			self.apriltagH12.apriltagH = self.apriltagHomography_H12_list
			self.apriltagH20.apriltagH = self.apriltagHomography_H20_list
			self.apriltagH21.apriltagH = self.apriltagHomography_H21_list
			self.apriltagH22.apriltagH = self.apriltagHomography_H22_list

			self.apriltagHomography_H00_pub.publish(self.apriltagH00)
			self.apriltagHomography_H01_pub.publish(self.apriltagH01)
			self.apriltagHomography_H02_pub.publish(self.apriltagH02)
			self.apriltagHomography_H10_pub.publish(self.apriltagH10)
			self.apriltagHomography_H11_pub.publish(self.apriltagH11)
			self.apriltagHomography_H12_pub.publish(self.apriltagH12)
			self.apriltagHomography_H20_pub.publish(self.apriltagH20)
			self.apriltagHomography_H21_pub.publish(self.apriltagH21)
			self.apriltagHomography_H22_pub.publish(self.apriltagH22)

			self.apriltagCorner_X1_list = []
			self.apriltagCorner_Y1_list = []
			self.apriltagCorner_X2_list = []
			self.apriltagCorner_Y2_list = []
			self.apriltagCorner_X3_list = []
			self.apriltagCorner_Y3_list = []
			self.apriltagCorner_X4_list = []
			self.apriltagCorner_Y4_list = []

			self.apriltagCornerX1.apriltagCorner = self.apriltagCorner_X1_list
			self.apriltagCornerY1.apriltagCorner = self.apriltagCorner_Y1_list
			self.apriltagCornerX2.apriltagCorner = self.apriltagCorner_X2_list
			self.apriltagCornerY2.apriltagCorner = self.apriltagCorner_Y2_list
			self.apriltagCornerX3.apriltagCorner = self.apriltagCorner_X3_list
			self.apriltagCornerY3.apriltagCorner = self.apriltagCorner_Y3_list
			self.apriltagCornerX4.apriltagCorner = self.apriltagCorner_X4_list
			self.apriltagCornerY4.apriltagCorner = self.apriltagCorner_Y4_list

			self.apriltagCorner_X1_pub.publish(self.apriltagCornerX1)
			self.apriltagCorner_Y1_pub.publish(self.apriltagCornerY1)
			self.apriltagCorner_X2_pub.publish(self.apriltagCornerX2)
			self.apriltagCorner_Y2_pub.publish(self.apriltagCornerY2)
			self.apriltagCorner_X3_pub.publish(self.apriltagCornerX3)
			self.apriltagCorner_Y3_pub.publish(self.apriltagCornerY3)
			self.apriltagCorner_X4_pub.publish(self.apriltagCornerX4)
			self.apriltagCorner_Y4_pub.publish(self.apriltagCornerY4)

		cv2.putText(
			self.cv_image, 
			"N AprilTag3: %d" % (len(result)), 
			(20, 50), 
			fontFace, 
			fontScale * 5, 
			(0, 0, 255), 
			thickness * 2, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(
			self.cv_image, 
			"Battery: %d" % (self.battery_percentage), 
			(20, 100), 
			fontFace, 
			fontScale * 5, 
			(0, 0, 255), 
			thickness * 2, 
			lineType, 
			bottomLeftOrigin)

		cv2.putText(
			self.cv_image, 
			"Height: %f" % (self.height_m), 
			(20, 150), 
			fontFace, 
			fontScale * 5, 
			(0, 0, 255), 
			thickness * 2, 
			lineType, 
			bottomLeftOrigin)

		if self.isApriltagDistance_received:
			cv2.putText(
				self.cv_image, 
				"Distance: %f" % (self.isApriltagDistance), 
				(20, 200), 
				fontFace, 
				fontScale * 5, 
				(0, 0, 255), 
				thickness * 2, 
				lineType, 
				bottomLeftOrigin)
		else:
			pass

	# Show the output frame
	def cbShowImage(self):

		self.cv_image_clone = imutils.resize(
					self.cv_image.copy(), 
					width=320
					)

		cv2.imshow("AprilTag Detection", self.cv_image_clone)
		cv2.waitKey(1)

	# Preview image + info
	def cbPreview(self):

		if self.image_received:
#			self.cbInfo()
			self.cbAprilTag()
			self.cbShowImage()
		else:
			rospy.logerr("No images recieved")

	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("AprilTag Detection Node [OFFLINE]...")
		cv2.destroyAllWindows()

if __name__ == '__main__':

	# Initialize
	rospy.init_node('camera_apriltag_detection', anonymous=False)
	camera = CameraAprilTag()

	r = rospy.Rate(10)

	# Camera preview
	while not rospy.is_shutdown():
		camera.cbPreview()
		r.sleep()

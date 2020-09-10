#!/usr/bin/env python

################################################################################
## {Description}: 
################################################################################
## Author: Khairul Izwan Bin Kamsani
## Version: {1}.{0}.{0}
## Email: {wansnap@gmail.com}
################################################################################

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
from std_msgs.msg import String, Float32
from sensor_msgs.msg import Image, CameraInfo, CompressedImage

from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

from tello_driver.msg import TelloStatus

from nav_msgs.msg import Odometry

from sensor_msgs.msg import Imu

from common_tello_application.msg import objCenter as objCoord

from common_tello_application.pid import PID
from common_tello_application.makesimpleprofile import map as mapped

from geometry_msgs.msg import Twist

import rospy

class CameraAprilTag:
	def __init__(self):

		self.bridge = CvBridge()
		self.image_received = False
		self.detector = apriltag.Detector()
		self.objectCoord = objCoord()
		self.panErrval = Float32()
		self.telloCmdVel = Twist()

		self.MAX_LIN_VEL = 2.00
		self.MAX_ANG_VEL = 2.00

		# set PID values for panning
		self.panP = 0.5
		self.panI = 0
		self.panD = 0

		# set PID values for tilting
		self.tiltP = 1
		self.tiltI = 0
		self.tiltD = 0

		# create a PID and initialize it
		self.panPID = PID(self.panP, self.panI, self.panD)
		self.tiltPID = PID(self.tiltP, self.tiltI, self.tiltD)

		self.panPID.initialize()
		self.tiltPID.initialize()

		rospy.logwarn("AprilTag Tracking Node [ONLINE]...")

		# rospy shutdown
		rospy.on_shutdown(self.cbShutdown)

#		# Subscribe to CompressedImage msg
#		self.telloImage_topic = "/tello/image_raw/compressed"
#		self.telloImage_sub = rospy.Subscriber(
#						self.telloImage_topic, 
#						CompressedImage, 
#						self.cbImage
#						)

		# Subscribe to CompressedImage msg
		self.telloCameraInfo_topic = "/tello/camera/camera_info"
		self.telloCameraInfo_sub = rospy.Subscriber(
						self.telloCameraInfo_topic, 
						CameraInfo, 
						self.cbCameraInfo
						)

#		# Subscribe to TelloStatus msg
#		self.telloStatus_topic = "/tello/status"
#		self.telloStatus_sub = rospy.Subscriber(
#					self.telloStatus_topic, 
#					TelloStatus, 
#					self.cbTelloStatus
#					)
#			
#		# Subscribe to Odometry msg
#		self.telloOdom_topic = "/tello/odom"
#		self.telloOdom_sub = rospy.Subscriber(
#					self.telloOdom_topic, 
#					Odometry, 
#					self.cbTelloOdometry
#					)

#		# Subscribe to PoseWithCovariance msg
#		self.telloIMU_topic = "/tello/imu"
#		self.telloIMU_sub = rospy.Subscriber(
#					self.telloIMU_topic, 
#					Imu, 
#					self.cbTelloIMU
#					)

		# Subscribe to objCenter msg
		self.objCoord_topic = "/objCoord"
		self.objCoord_sub = rospy.Subscriber(
					self.objCoord_topic, 
					objCoord, 
					self.cbObjCoord
					)

		# Publish to Float32 msg
		self.panErr_topic = "/panErr"
		self.panErr_pub = rospy.Publisher(
					self.panErr_topic, 
					Float32, 
					queue_size=10
					)

		# Publish to Twist msg
		self.telloCmdVel_topic = "/tello/cmd_vel"
		self.telloCmdVel_pub = rospy.Publisher(
					self.telloCmdVel_topic, 
					Twist, 
					queue_size=10
					)

		# Allow up to one second to connection
		rospy.sleep(1)

#	# Convert image to OpenCV format
#	def cbImage(self, msg):

#		try:
#			# direct conversion to cv2
#			self.cv_image = self.bridge.compressed_imgmsg_to_cv2(
#								msg, 
#								"bgr8"
#								)
#		except CvBridgeError as e:
#			print(e)

#		if self.cv_image is not None:
#			self.image_received = True
#		else:
#			self.image_received = False

	# Convert image to OpenCV format
	def cbCameraInfo(self, msg):

		self.imgWidth = msg.width
		self.imgHeight = msg.height
		
	# Convert image to OpenCV format
	def cbObjCoord(self, msg):

		self.objectCoordX = msg.centerX
		self.objectCoordY = msg.centerY

#	# Get TelloIMU info
#	def cbTelloIMU(self, msg):
#	
#		self.orientationXIMU = msg.orientation.x
#		self.orientationYIMU = msg.orientation.y
#		self.orientationZIMU = msg.orientation.z
#		self.orientationWIMU = msg.orientation.w
#		
#		self.angularXIMU = msg.angular_velocity.x
#		self.angularYIMU = msg.angular_velocity.y
#		self.angularZIMU = msg.angular_velocity.z
#		
#		self.linearXIMU = msg.linear_acceleration.x
#		self.linearYIMU = msg.linear_acceleration.y
#		self.linearZIMU = msg.linear_acceleration.z
#		
#	# Get TelloOdometry info
#	def cbTelloOdometry(self, msg):
#	
#		self.poseX = msg.pose.pose.position.x
#		self.poseY = msg.pose.pose.position.y
#		self.poseZ = msg.pose.pose.position.z
#		self.orientationX = msg.pose.pose.orientation.x
#		self.orientationY = msg.pose.pose.orientation.y
#		self.orientationZ = msg.pose.pose.orientation.z
#		self.orientationW = msg.pose.pose.orientation.w
#		
#		self.linearX = msg.twist.twist.linear.x
#		self.linearY = msg.twist.twist.linear.y
#		self.linearZ = msg.twist.twist.linear.z
#		self.angularX = msg.twist.twist.angular.x
#		self.angularY = msg.twist.twist.angular.y
#		self.angularZ = msg.twist.twist.angular.z

#	# Get TelloStatus info
#	def cbTelloStatus(self, msg):

#		# Non-negative; calibrated to takeoff altitude; auto-calib if 
#		# falls below takeoff height; inaccurate near ground
#		self.height_m = msg.height_m

#		self.speed_northing_mps = msg.speed_northing_mps
#		self.speed_easting_mps = msg.speed_easting_mps
#		self.speed_horizontal_mps = msg.speed_horizontal_mps
#		self.speed_vertical_mps = msg.speed_vertical_mps

#		self.flight_time_sec = msg.flight_time_sec

#		self.imu_state = msg.imu_state
#		self.pressure_state = msg.pressure_state
#		self.down_visual_state = msg.down_visual_state
#		self.power_state = msg.power_state
#		self.battery_state = msg.battery_state
#		self.gravity_state = msg.gravity_state
#		self.wind_state = msg.wind_state

#		self.imu_calibration_state = msg.imu_calibration_state
#		self.battery_percentage = msg.battery_percentage
#		self.drone_fly_time_left_sec = msg.drone_fly_time_left_sec
#		self.drone_battery_left_sec = msg.drone_battery_left_sec

#		self.is_flying = msg.is_flying
#		self.is_on_ground = msg.is_on_ground
#		# is_em_open True in flight, False when landed
#		self.is_em_open = msg.is_em_open
#		self.is_drone_hover = msg.is_drone_hover
#		self.is_outage_recording = msg.is_outage_recording
#		self.is_battery_low = msg.is_battery_low
#		self.is_battery_lower = msg.is_battery_lower
#		self.is_factory_mode = msg.is_factory_mode

#		# flymode=1: landed; =6: flying
#		self.fly_mode = msg.fly_mode
#		self.throw_takeoff_timer_sec = msg.throw_takeoff_timer_sec
#		self.camera_state = msg.camera_state

#		self.electrical_machinery_state = msg.electrical_machinery_state

#		self.front_in = msg.front_in
#		self.front_out = msg.front_out
#		self.front_lsc = msg.front_lsc

#		self.temperature_height_m = msg.temperature_height_m

#		self.cmd_roll_ratio = msg.cmd_roll_ratio
#		self.cmd_pitch_ratio = msg.cmd_pitch_ratio
#		self.cmd_yaw_ratio = msg.cmd_yaw_ratio
#		self.cmd_vspeed_ratio = msg.cmd_vspeed_ratio
#		self.cmd_fast_mode = msg.cmd_fast_mode

#	# Image information callback
#	def cbInfo(self):

#		fontFace = cv2.FONT_HERSHEY_PLAIN
#		fontScale = 2
#		color = (255, 255, 255)
#		colorPose = (0, 0, 255)
#		colorIMU = (255, 0, 255)
#		thickness = 2
#		lineType = cv2.LINE_AA
#		bottomLeftOrigin = False # if True (text upside down)
#		
#		cv2.putText(self.cv_image, "(W: %d, H: %d)" % (self.imgWidth, 
#			self.imgHeight), 
#			(10, 30), 
#			fontFace, 
#			fontScale, 
#			color, 
#			thickness, 
#			lineType, 
#			bottomLeftOrigin
#			)

#		cv2.putText(self.cv_image, "(W: %d, H: %d)" % 
#			(self.objectCoordX, self.objectCoordY), 
#			(10, 60), 
#			fontFace, 
#			fontScale, 
#			color, 
#			thickness, 
#			lineType, 
#			bottomLeftOrigin
#			)

###		self.timestr = time.strftime("%Y%m%d-%H:%M:%S")

##		# Status
##		cv2.putText(self.cv_image, "height_m: %.4f" % (self.height_m), (10, 10), 
##			fontFace, fontScale, color, thickness, lineType, 
##			bottomLeftOrigin)
##		cv2.putText(self.cv_image, "speed_northing_mps: %.4f" % (self.speed_northing_mps), (10, 30), 
##			fontFace, fontScale, color, thickness, lineType, 
##			bottomLeftOrigin)
##		cv2.putText(self.cv_image, "speed_easting_mps: %.4f" % (self.speed_easting_mps), (10, 40), 
##			fontFace, fontScale, color, thickness, lineType, 
##			bottomLeftOrigin)
##		cv2.putText(self.cv_image, "speed_horizontal_mps: %.4f" % (self.speed_horizontal_mps), (10, 50), 
##			fontFace, fontScale, color, thickness, lineType, 
##			bottomLeftOrigin)
##		cv2.putText(self.cv_image, "speed_vertical_mps: %.4f" % (self.speed_vertical_mps), (10, 60), 
##			fontFace, fontScale, color, thickness, lineType, 
##			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "flight_time_sec: %.4f" % (self.flight_time_sec), (10, 80), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "imu_state: %s" % (self.imu_state), (10, 100), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)			
###		cv2.putText(self.cv_image, "pressure_state: %s" % (self.pressure_state), (10, 110), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)						
###		cv2.putText(self.cv_image, "down_visual_state: %s" % (self.down_visual_state), (10, 120), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "pressure_state: %s" % (self.pressure_state), (10, 130), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "power_state: %s" % (self.power_state), (10, 140), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "battery_state: %s" % (self.battery_state), (10, 150), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "gravity_state: %s" % (self.gravity_state), (10, 160), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "wind_state: %s" % (self.wind_state), (10, 170), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "imu_calibration_state: %d" % (self.imu_calibration_state), (10, 190), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
##		cv2.putText(self.cv_image, "battery_percentage: %d" % (self.battery_percentage), (10, 200), 
##			fontFace, fontScale, color, thickness, lineType, 
##			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "drone_fly_time_left_sec: %.4f" % (self.drone_fly_time_left_sec), (10, 210), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "drone_battery_left_sec: %.4f" % (self.drone_battery_left_sec), (10, 220), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
##		cv2.putText(self.cv_image, "is_flying: %s" % (self.is_flying), (10, 240), 
##			fontFace, fontScale, color, thickness, lineType, 
##			bottomLeftOrigin)
##		cv2.putText(self.cv_image, "is_on_ground: %s" % (self.is_on_ground), (10, 250), 
##			fontFace, fontScale, color, thickness, lineType, 
##			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "is_em_open: %s" % (self.is_em_open), (10, 260), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "is_drone_hover: %s" % (self.is_drone_hover), (10, 270), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "is_outage_recording: %s" % (self.is_outage_recording), (10, 280), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "is_battery_low: %s" % (self.is_battery_low), (10, 290), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "is_battery_lower: %s" % (self.is_battery_lower), (10, 300), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "is_factory_mode: %s" % (self.is_factory_mode), (10, 310), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
##		cv2.putText(self.cv_image, "fly_mode: %d" % (self.fly_mode), (10, 330), 
##			fontFace, fontScale, color, thickness, lineType, 
##			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "throw_takeoff_timer_sec: %.4f" % (self.throw_takeoff_timer_sec), (10, 340), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "camera_state: %d" % (self.camera_state), (10, 350), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "electrical_machinery_state: %d" % (self.electrical_machinery_state), (10, 370), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "front_in: %s" % (self.front_in), (10, 390), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "front_out: %s" % (self.front_out), (10, 400), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "front_lsc: %s" % (self.front_lsc), (10, 410), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "temperature_height_m: %.4f" % (self.temperature_height_m), (10, 430), 
###			fontFace, fontScale, color, thickness, lineType, 
###			bottomLeftOrigin)
##		cv2.putText(self.cv_image, "cmd_roll_ratio: %.4f" % (self.cmd_roll_ratio), (10, 450), 
##			fontFace, fontScale, color, thickness, lineType, 
##			bottomLeftOrigin)
##		cv2.putText(self.cv_image, "cmd_pitch_ratio: %.4f" % (self.cmd_pitch_ratio), (10, 460), 
##			fontFace, fontScale, color, thickness, lineType, 
##			bottomLeftOrigin)
##		cv2.putText(self.cv_image, "cmd_yaw_ratio: %.4f" % (self.cmd_yaw_ratio), (10, 470), 
##			fontFace, fontScale, color, thickness, lineType, 
##			bottomLeftOrigin)
##		cv2.putText(self.cv_image, "cmd_vspeed_ratio: %.4f" % (self.cmd_vspeed_ratio), (10, 480), 
##			fontFace, fontScale, color, thickness, lineType, 
##			bottomLeftOrigin)
##		cv2.putText(self.cv_image, "cmd_fast_mode: %d" % (self.cmd_fast_mode), (10, 490), 
##			fontFace, fontScale, color, thickness, lineType, 
##			bottomLeftOrigin)

##		# ODOM
###		cv2.putText(self.cv_image, "ODOMPose: X:%.4f, Y:%.4f, Y:%.4f" % (self.poseX, self.poseY, self.poseZ), (300, 10), 
###			fontFace, fontScale, colorPose, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "ODOMOrientation: X:%.4f, Y:%.4f, Z:%.4f, W:%.4f" % (self.orientationX, self.orientationY, self.orientationZ, self.orientationW), (300, 20), 
###			fontFace, fontScale, colorPose, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "ODOMvelLinear: X:%.4f, Y:%.4f, Z:%.4f" % (self.linearX, self.linearY, self.linearZ), (300, 30), 
###			fontFace, fontScale, colorPose, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "ODOMvelAngular: X:%.4f, Y:%.4f, Z:%.4f" % (self.angularX, self.angularY, self.angularZ), (300, 40), 
###			fontFace, fontScale, colorPose, thickness, lineType, 
###			bottomLeftOrigin)

##		# IMU
###		cv2.putText(self.cv_image, "IMUOrientation: X:%.4f, Y:%.4f, Z:%.4f, W:%.4f" % (self.orientationXIMU, self.orientationYIMU, self.orientationZIMU, self.orientationWIMU), (300, 60), 
###			fontFace, fontScale, colorIMU, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "IMUvelAngular: X:%.4f, Y:%.4f, Z:%.4f" % (self.angularXIMU, self.angularYIMU, self.angularZIMU), (300, 70), 
###			fontFace, fontScale, colorIMU, thickness, lineType, 
###			bottomLeftOrigin)
###		cv2.putText(self.cv_image, "IMUvelLinear: X:%.4f, Y:%.4f, Z:%.4f" % (self.linearXIMU, self.linearYIMU, self.linearZIMU), (300, 80), 
###			fontFace, fontScale, colorIMU, thickness, lineType, 
###			bottomLeftOrigin)

	def cbAprilTag(self):
		
#		fontFace = cv2.FONT_HERSHEY_PLAIN
#		fontScale = 0.7
#		color = (255, 255, 255)
#		colorPose = (0, 0, 255)
#		colorIMU = (255, 0, 255)
#		thickness = 1
#		lineType = cv2.LINE_AA
#		bottomLeftOrigin = False # if True (text upside down)
#		
#		cv_image_gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
#		
#		result = self.detector.detect(cv_image_gray)
#		
##		rospy.loginfo(len(result))

#		if len(result) != 0:
##			for i in range(len(result)):
##				rospy.loginfo("Detect ID: %d" % (result[i][1]))

#			self.objectCoord.centerX = int(result[0][6][0])
#			self.objectCoord.centerY = int(result[0][6][1])

#			cv2.putText(
#				self.cv_image, 
#				"ID: %d" % (result[0][1]), 
#				(int(result[0][6][0]) - 20, int(result[0][6][1]) - 20), 
#				fontFace, 
#				fontScale, 
#				color, 
#				thickness, 
#				lineType, 
#				bottomLeftOrigin)

#			cv2.line(
#				self.cv_image, 
#				(int(result[0][7][0][0]), int(result[0][7][0][1])), 
#				(int(result[0][7][1][0]), int(result[0][7][1][1])), 
#				(0, 0, 255), 
#				3)

#			cv2.line(
#				self.cv_image, 
#				(int(result[0][7][0][0]), int(result[0][7][0][1])), 
#				(int(result[0][7][3][0]), int(result[0][7][3][1])), 
#				(0, 255, 0), 
#				3)

#			cv2.line(
#				self.cv_image, 
#				(int(result[0][7][1][0]), int(result[0][7][1][1])), 
#				(int(result[0][7][2][0]), int(result[0][7][2][1])), 
#				(255, 0, 0), 
#				3)

#			cv2.line(
#				self.cv_image, 
#				(int(result[0][7][2][0]), int(result[0][7][2][1])), 
#				(int(result[0][7][3][0]), int(result[0][7][3][1])), 
#				(255, 0, 0), 
#				3)

#			cv2.circle(
#				self.cv_image, 
#				(int(result[0][6][0]), int(result[0][6][1])), 
#				5, 
#				(255, 0, 0), 
#				2)
#		
###			print("Corners: {}".format(result[0][7]))
###			print("Corners: {}".format(result[0][7][0]))
###			print("Corners: {}".format(result[0][7][0][0]))
#		else:
#			self.objectCoord.centerX = self.imgWidth
#			self.objectCoord.centerY = self.imgHeight

#		self.objCoord_pub.publish(self.objectCoord)

		self.cbPIDerr()

#	# Show the output frame
#	def cbShowImage(self):

#		self.cv_image_clone = imutils.resize(
#					self.cv_image.copy(), 
#					width=320
#					)

#		cv2.imshow("AprilTag Tracking", self.cv_image_clone)
#		cv2.waitKey(1)

#	# Preview image + info
#	def cbPreview(self):

#		if self.image_received:
#			self.cbInfo()
#			self.cbAprilTag()
#			self.cbShowImage()
#		else:
#			rospy.logerr("No images recieved")

#	# Publish to objCenter msg
#	def pubObjCoord(self):

#		self.objectCoord.centerX = self.cv_image_height
#		self.objectCoord.centerY = self.cv_image_width

#		self.objCoord_pub.publish(self.objectCoord)

	# show information callback
	def cbPIDerr(self):

		self.panErr, self.panOut = self.cbPIDprocess(self.panPID, self.objectCoordX, self.imgWidth // 2)
		self.tiltErr, self.tiltOut = self.cbPIDprocess(self.tiltPID, self.objectCoordY, self.imgHeight // 2)
		
#		rospy.loginfo([self.panErr, self.panOut])

#		self.panErrval.data = self.panOut
#		self.panErr_pub.publish(self.panErrval)

	def cbPIDprocess(self, pid, objCoord, centerCoord):

		# calculate the error
		error = centerCoord - objCoord

		# update the value
		output = pid.update(error)

		return error, output

	def cbCallErr(self):
		self.cbAprilTag()

#		# set self.telloCmdVel.linear.x
#		if self.panOut > 0:	# positive error
#			self.telloCmdVel.linear.x = self.map(self.panOut, 0, self.panOut, 0, -self.MAX_LIN_VEL)
#		elif self.panOut < 0:	# negative error
#			self.telloCmdVel.linear.x = self.map(self.panOut, 0, self.panOut, 0, self.MAX_LIN_VEL)

		panSpeed = mapped(abs(self.panOut), 0, self.imgWidth // 2, 0, self.MAX_LIN_VEL)
		tiltSpeed = mapped(abs(self.tiltOut), 0, self.imgHeight // 2, 0, self.MAX_LIN_VEL)

		# print an error
#		rospy.loginfo([self.panErr, self.panOut, panSpeed])
#		rospy.loginfo([self.tiltErr, self.tiltOut, tiltSpeed])
		
#		# set self.telloCmdVel.linear.z
#		if self.tiltOut > 0:	# positive error
#			# TODO:
#		elif self.tiltOut < 0:	# negative error
#			# TODO
		
		if self.panOut < 0:
			self.telloCmdVel.linear.x = panSpeed
		elif self.panOut > 0:
			self.telloCmdVel.linear.x = -panSpeed
		else:
			self.telloCmdVel.linear.x = 0
			
		if self.tiltOut > 0:
			self.telloCmdVel.linear.z = tiltSpeed
		elif self.tiltOut < 0:
			self.telloCmdVel.linear.z = -tiltSpeed
		else:
			self.telloCmdVel.linear.z = 0

#		if self.panErr > 0:
#			self.telloCmdVel.linear.x = -0.5
#			self.telloCmdVel.linear.y = 0.0
#			self.telloCmdVel.linear.z = 0.0
#		
#			self.telloCmdVel.angular.x = 0.0
#			self.telloCmdVel.angular.y = 0.0
#			self.telloCmdVel.angular.z = 0.0

#		elif self.panErr < 0:
#			self.telloCmdVel.linear.x = 0.5
#			self.telloCmdVel.linear.y = 0.0
#			self.telloCmdVel.linear.z = 0.0
#		
#			self.telloCmdVel.angular.x = 0.0
#			self.telloCmdVel.angular.y = 0.0
#			self.telloCmdVel.angular.z = 0.0

#		elif self.tiltErr > 0:
#			self.telloCmdVel.linear.x = 0.0
#			self.telloCmdVel.linear.y = 0.0
#			self.telloCmdVel.linear.z = 0.5
#		
#			self.telloCmdVel.angular.x = 0.0
#			self.telloCmdVel.angular.y = 0.0
#			self.telloCmdVel.angular.z = 0.0

#		elif self.tiltErr < 0:
#			self.telloCmdVel.linear.x = 0.0
#			self.telloCmdVel.linear.y = 0.0
#			self.telloCmdVel.linear.z = -0.5
#		
#			self.telloCmdVel.angular.x = 0.0
#			self.telloCmdVel.angular.y = 0.0
#			self.telloCmdVel.angular.z = 0.0

#		else:
#			self.telloCmdVel.linear.x = 0.0
#			self.telloCmdVel.linear.y = 0.0
#			self.telloCmdVel.linear.z = 0.0
#		
#			self.telloCmdVel.angular.x = 0.0
#			self.telloCmdVel.angular.y = 0.0
#			self.telloCmdVel.angular.z = 0.0

		self.telloCmdVel.linear.y = 0.0
	
		self.telloCmdVel.angular.x = 0.0
		self.telloCmdVel.angular.y = 0.0
		self.telloCmdVel.angular.z = 0.0
		self.telloCmdVel_pub.publish(self.telloCmdVel)


	# rospy shutdown callback
	def cbShutdown(self):

		rospy.logerr("AprilTag Tracking Node [OFFLINE]...")
#		cv2.destroyAllWindows()

if __name__ == '__main__':

	# Initialize
	rospy.init_node('camera_apriltag_tracking', anonymous=False)
	camera = CameraAprilTag()
	
#	r = rospy.Rate(10)
	
	# Camera preview
	while not rospy.is_shutdown():
		camera.cbCallErr()
#		r.sleep()

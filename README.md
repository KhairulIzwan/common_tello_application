# common_tello_application
# About
A project to understand and controlling the **tello** with **ROS** and **Python**

# System Requirement
1. Ubuntu 16.04
2. ROS Kinetic

# Required Packages/Library
1. Tello Driver: 
	1. https://github.com/KhairulIzwan/tello_driver.git

	**installation**
	1. cd ~/catkin_ws/src
	2. git clone https://github.com/KhairulIzwan/tello_driver.git
	3. cd ~/catkin_ws
	4. catkin_make

2. AprilTag:
	1. https://pypi.org/project/apriltag/

	**installation**
	1. python -m pip install apriltag

3. Imutils:
	1. https://pypi.org/project/imutils/

	**installation**
	1. python -m pip install imutils

<!--# Important!-->
<!--**Tello Driver**-->
<!--1. Open tello_node.launch: rosed tello_driver tello_node.launch-->
<!--2. Edit this line:-->
<!--```-->
<!--<node	pkg="image_transport" -->
<!--	name="image_compressed" -->
<!--	type="republish" -->
<!--	args="h264 in:=image_raw compressed out:=image_raw" />-->
<!--```-->

**AprilTag3**
# TODO:

# How it works?
## [1] tello_teleop_key.py
Controlling the tello drone using keyboard

```
Control Your Tello Drone!
---------------------------
Moving around:
		w					i
	a	s	d			j	k	l

w/s : increase/decrease linear (y) velocity
a/d : increase/decrease linear (x) velocity
i/k : increase/decrease linear (z) velocity
a/d : increase/decrease angular velocity

v : takeoff
b : land
space key : force stop

CTRL-C to quit
```

1. roslaunch tello_driver tello_node.launch

 ```
Node [/tello/tello_driver_node]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tello/camera/camera_info [sensor_msgs/CameraInfo]
 * /tello/image_raw/h264 [sensor_msgs/CompressedImage]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]
 * /tello/tello_driver_node/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /tello/tello_driver_node/parameter_updates [dynamic_reconfigure/Config]

Subscriptions: 
 * /tello/cmd_vel [geometry_msgs/Twist]
 * /tello/emergency [unknown type]
 * /tello/fast_mode [unknown type]
 * /tello/flattrim [unknown type]
 * /tello/flip [unknown type]
 * /tello/land [std_msgs/Empty]
 * /tello/manual_takeoff [unknown type]
 * /tello/palm_land [unknown type]
 * /tello/takeoff [std_msgs/Empty]
 * /tello/throw_takeoff [unknown type]
 * /tello/video_mode [unknown type]

Services: 
 * /tello/tello_driver_node/get_loggers
 * /tello/tello_driver_node/set_logger_level
 * /tello/tello_driver_node/set_parameters
 ```

<!--contacting node http://192.168.10.4:44357/ ...-->
<!--Pid: 22495-->
<!--Connections:-->
<!-- * topic: /tello/image_raw/h264-->
<!--    * to: /tello/image_compressed-->
<!--    * direction: outbound-->
<!--    * transport: TCPROS-->
<!-- * topic: /rosout-->
<!--    * to: /rosout-->
<!--    * direction: outbound-->
<!--    * transport: TCPROS-->
<!-- * topic: /tello/land-->
<!--    * to: /tello_teleop (http://192.168.10.4:46683/)-->
<!--    * direction: inbound-->
<!--    * transport: TCPROS-->
<!-- * topic: /tello/takeoff-->
<!--    * to: /tello_teleop (http://192.168.10.4:46683/)-->
<!--    * direction: inbound-->
<!--    * transport: TCPROS-->
<!-- * topic: /tello/cmd_vel-->
<!--    * to: /tello_teleop (http://192.168.10.4:46683/)-->
<!--    * direction: inbound-->
<!--    * transport: TCPROS-->
 
2. rosrun common_tello_application tello_teleop_key.py

```
Node [/tello_teleop]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tello/cmd_vel [geometry_msgs/Twist]
 * /tello/land [std_msgs/Empty]
 * /tello/takeoff [std_msgs/Empty]

Subscriptions: None

Services: 
 * /tello_teleop/get_loggers
 * /tello_teleop/set_logger_level
```

<!--contacting node http://192.168.10.4:46683/ ...-->
<!--Pid: 22639-->
<!--Connections:-->
<!-- * topic: /tello/land-->
<!--    * to: /tello/tello_driver_node-->
<!--    * direction: outbound-->
<!--    * transport: TCPROS-->
<!-- * topic: /rosout-->
<!--    * to: /rosout-->
<!--    * direction: outbound-->
<!--    * transport: TCPROS-->
<!-- * topic: /tello/takeoff-->
<!--    * to: /tello/tello_driver_node-->
<!--    * direction: outbound-->
<!--    * transport: TCPROS-->
<!-- * topic: /tello/cmd_vel-->
<!--    * to: /tello/tello_driver_node-->
<!--    * direction: outbound-->
<!--    * transport: TCPROS-->

## [2] camera_preview.py
[x] Preview an image stream from tello

1. roslaunch tello_driver tello_node.launch

 ```
Node [/tello/tello_driver_node]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tello/camera/camera_info [sensor_msgs/CameraInfo]
 * /tello/image_raw/h264 [sensor_msgs/CompressedImage]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]
 * /tello/tello_driver_node/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /tello/tello_driver_node/parameter_updates [dynamic_reconfigure/Config]

Subscriptions: 
 * /tello/cmd_vel [geometry_msgs/Twist]
 * /tello/emergency [unknown type]
 * /tello/fast_mode [unknown type]
 * /tello/flattrim [unknown type]
 * /tello/flip [unknown type]
 * /tello/land [std_msgs/Empty]
 * /tello/manual_takeoff [unknown type]
 * /tello/palm_land [unknown type]
 * /tello/takeoff [std_msgs/Empty]
 * /tello/throw_takeoff [unknown type]
 * /tello/video_mode [unknown type]

Services: 
 * /tello/tello_driver_node/get_loggers
 * /tello/tello_driver_node/set_logger_level
 * /tello/tello_driver_node/set_parameters
 ```

<!--contacting node http://192.168.10.4:44357/ ...-->
<!--Pid: 22495-->
<!--Connections:-->
<!-- * topic: /tello/image_raw/h264-->
<!--    * to: /tello/image_compressed-->
<!--    * direction: outbound-->
<!--    * transport: TCPROS-->
<!-- * topic: /rosout-->
<!--    * to: /rosout-->
<!--    * direction: outbound-->
<!--    * transport: TCPROS-->
<!-- * topic: /tello/land-->
<!--    * to: /tello_teleop (http://192.168.10.4:46683/)-->
<!--    * direction: inbound-->
<!--    * transport: TCPROS-->
<!-- * topic: /tello/takeoff-->
<!--    * to: /tello_teleop (http://192.168.10.4:46683/)-->
<!--    * direction: inbound-->
<!--    * transport: TCPROS-->
<!-- * topic: /tello/cmd_vel-->
<!--    * to: /tello_teleop (http://192.168.10.4:46683/)-->
<!--    * direction: inbound-->
<!--    * transport: TCPROS-->

2. rosrun common_tello_application camera_preview.py

```
Node [/camera_preview]
Publications: 
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /tello/image_raw/compressed [sensor_msgs/CompressedImage]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]

Services: 
 * /camera_preview/get_loggers
 * /camera_preview/set_logger_level


contacting node http://192.168.10.4:41771/ ...
Pid: 22984
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /tello/odom
    * to: /tello/tello_driver_node (http://192.168.10.4:44357/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/status
    * to: /tello/tello_driver_node (http://192.168.10.4:44357/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/imu
    * to: /tello/tello_driver_node (http://192.168.10.4:44357/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/image_raw/compressed
    * to: /tello/image_compressed (http://192.168.10.4:39501/)
    * direction: inbound
    * transport: TCPROS
```

## [2] camera_apriltag_detection.py
[x] Detect an AprilTag3 to get useful information

1. roslaunch tello_driver tello_node.launch

 ```
Node [/tello/tello_driver_node]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tello/camera/camera_info [sensor_msgs/CameraInfo]
 * /tello/image_raw/h264 [sensor_msgs/CompressedImage]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]
 * /tello/tello_driver_node/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /tello/tello_driver_node/parameter_updates [dynamic_reconfigure/Config]

Subscriptions: 
 * /tello/cmd_vel [geometry_msgs/Twist]
 * /tello/emergency [unknown type]
 * /tello/fast_mode [unknown type]
 * /tello/flattrim [unknown type]
 * /tello/flip [unknown type]
 * /tello/land [std_msgs/Empty]
 * /tello/manual_takeoff [unknown type]
 * /tello/palm_land [unknown type]
 * /tello/takeoff [std_msgs/Empty]
 * /tello/throw_takeoff [unknown type]
 * /tello/video_mode [unknown type]

Services: 
 * /tello/tello_driver_node/get_loggers
 * /tello/tello_driver_node/set_logger_level
 * /tello/tello_driver_node/set_parameters


contacting node http://192.168.10.4:44357/ ...
Pid: 22495
Connections:
 * topic: /tello/image_raw/h264
    * to: /tello/image_compressed
    * direction: outbound
    * transport: TCPROS
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /tello/land
    * to: /tello_teleop (http://192.168.10.4:46683/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/takeoff
    * to: /tello_teleop (http://192.168.10.4:46683/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/cmd_vel
    * to: /tello_teleop (http://192.168.10.4:46683/)
    * direction: inbound
    * transport: TCPROS
 ```

2. rosrun common_tello_application camera_apriltag_detection.py

```
Node [/camera_apriltag_detection]
Publications: 
 * /isApriltag [std_msgs/Bool]
 * /isApriltag/Center/X [common_tello_application/apriltagC]
 * /isApriltag/Center/Y [common_tello_application/apriltagC]
 * /isApriltag/Corner/X1 [common_tello_application/apriltagCorner]
 * /isApriltag/Corner/X2 [common_tello_application/apriltagCorner]
 * /isApriltag/Corner/X3 [common_tello_application/apriltagCorner]
 * /isApriltag/Corner/X4 [common_tello_application/apriltagCorner]
 * /isApriltag/Corner/Y1 [common_tello_application/apriltagCorner]
 * /isApriltag/Corner/Y2 [common_tello_application/apriltagCorner]
 * /isApriltag/Corner/Y3 [common_tello_application/apriltagCorner]
 * /isApriltag/Corner/Y4 [common_tello_application/apriltagCorner]
 * /isApriltag/Homography/H00 [common_tello_application/apriltagH]
 * /isApriltag/Homography/H01 [common_tello_application/apriltagH]
 * /isApriltag/Homography/H02 [common_tello_application/apriltagH]
 * /isApriltag/Homography/H10 [common_tello_application/apriltagH]
 * /isApriltag/Homography/H11 [common_tello_application/apriltagH]
 * /isApriltag/Homography/H12 [common_tello_application/apriltagH]
 * /isApriltag/Homography/H20 [common_tello_application/apriltagH]
 * /isApriltag/Homography/H21 [common_tello_application/apriltagH]
 * /isApriltag/Homography/H22 [common_tello_application/apriltagH]
 * /isApriltag/N [common_tello_application/apriltagN]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /isApriltag/Corner/Distance [unknown type]
 * /tello/image_raw/compressed [sensor_msgs/CompressedImage]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]

Services: 
 * /camera_apriltag_detection/get_loggers
 * /camera_apriltag_detection/set_logger_level


contacting node http://192.168.10.4:35753/ ...
Pid: 5749
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /tello/odom
    * to: /tello/tello_driver_node (http://192.168.10.4:32969/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/status
    * to: /tello/tello_driver_node (http://192.168.10.4:32969/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/imu
    * to: /tello/tello_driver_node (http://192.168.10.4:32969/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/image_raw/compressed
    * to: /tello/image_compressed (http://192.168.10.4:35055/)
    * direction: inbound
    * transport: TCPROS

```

## [4] camera_apriltag_takeoff_land.py
Autonomously takeoff and land based on apriltag 0: Land, 1: Takeoff

1. roslaunch tello_driver tello_node.launch

 ```
Node [/tello/tello_driver_node]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tello/camera/camera_info [sensor_msgs/CameraInfo]
 * /tello/image_raw/h264 [sensor_msgs/CompressedImage]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]
 * /tello/tello_driver_node/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /tello/tello_driver_node/parameter_updates [dynamic_reconfigure/Config]

Subscriptions: 
 * /tello/cmd_vel [geometry_msgs/Twist]
 * /tello/emergency [unknown type]
 * /tello/fast_mode [unknown type]
 * /tello/flattrim [unknown type]
 * /tello/flip [unknown type]
 * /tello/land [std_msgs/Empty]
 * /tello/manual_takeoff [unknown type]
 * /tello/palm_land [unknown type]
 * /tello/takeoff [std_msgs/Empty]
 * /tello/throw_takeoff [unknown type]
 * /tello/video_mode [unknown type]

Services: 
 * /tello/tello_driver_node/get_loggers
 * /tello/tello_driver_node/set_logger_level
 * /tello/tello_driver_node/set_parameters


contacting node http://192.168.10.4:44357/ ...
Pid: 22495
Connections:
 * topic: /tello/image_raw/h264
    * to: /tello/image_compressed
    * direction: outbound
    * transport: TCPROS
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /tello/land
    * to: /tello_teleop (http://192.168.10.4:46683/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/takeoff
    * to: /tello_teleop (http://192.168.10.4:46683/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/cmd_vel
    * to: /tello_teleop (http://192.168.10.4:46683/)
    * direction: inbound
    * transport: TCPROS
 ```
 
2. rosrun common_tello_application camera_apriltag_takeoff_land.py

```
Node [/camera_apriltag_takeoff_land]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tello/land [std_msgs/Empty]
 * /tello/takeoff [std_msgs/Empty]

Subscriptions: 
 * /isApriltag [std_msgs/Bool]
 * /isApriltag/N [common_tello_application/apriltagN]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]

Services: 
 * /camera_apriltag_takeoff_land/get_loggers
 * /camera_apriltag_takeoff_land/set_logger_level


contacting node http://192.168.10.4:34283/ ...
Pid: 3465
Connections:
 * topic: /tello/land
    * to: /tello/tello_driver_node
    * direction: outbound
    * transport: TCPROS
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /tello/takeoff
    * to: /tello/tello_driver_node
    * direction: outbound
    * transport: TCPROS
 * topic: /tello/odom
    * to: /tello/tello_driver_node (http://192.168.10.4:33715/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/status
    * to: /tello/tello_driver_node (http://192.168.10.4:33715/)
    * direction: inbound
    * transport: TCPROS
 * topic: /isApriltag
    * to: /camera_apriltag_detection (http://192.168.10.4:38191/)
    * direction: inbound
    * transport: TCPROS
 * topic: /isApriltag/N
    * to: /camera_apriltag_detection (http://192.168.10.4:38191/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/imu
    * to: /tello/tello_driver_node (http://192.168.10.4:33715/)
    * direction: inbound
    * transport: TCPROS
```

## [5] camera_apriltag_center.py
Detect, recognize apriltag and publish the center

1. roslaunch tello_driver tello_node.launch

 ```
Node [/tello/tello_driver_node]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tello/camera/camera_info [sensor_msgs/CameraInfo]
 * /tello/image_raw/h264 [sensor_msgs/CompressedImage]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]
 * /tello/tello_driver_node/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /tello/tello_driver_node/parameter_updates [dynamic_reconfigure/Config]

Subscriptions: 
 * /tello/cmd_vel [geometry_msgs/Twist]
 * /tello/emergency [unknown type]
 * /tello/fast_mode [unknown type]
 * /tello/flattrim [unknown type]
 * /tello/flip [unknown type]
 * /tello/land [std_msgs/Empty]
 * /tello/manual_takeoff [unknown type]
 * /tello/palm_land [unknown type]
 * /tello/takeoff [std_msgs/Empty]
 * /tello/throw_takeoff [unknown type]
 * /tello/video_mode [unknown type]

Services: 
 * /tello/tello_driver_node/get_loggers
 * /tello/tello_driver_node/set_logger_level
 * /tello/tello_driver_node/set_parameters


contacting node http://192.168.10.4:44357/ ...
Pid: 22495
Connections:
 * topic: /tello/image_raw/h264
    * to: /tello/image_compressed
    * direction: outbound
    * transport: TCPROS
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /tello/land
    * to: /tello_teleop (http://192.168.10.4:46683/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/takeoff
    * to: /tello_teleop (http://192.168.10.4:46683/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/cmd_vel
    * to: /tello_teleop (http://192.168.10.4:46683/)
    * direction: inbound
    * transport: TCPROS
 ```
 
2. rosrun common_tello_application camera_apriltag_center.py

```
Node [/camera_apriltag_center]
Publications: 
 * /isApriltag/objCoord [common_tello_application/objCenter]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /isApriltag [std_msgs/Bool]
 * /isApriltag/Center/X [common_tello_application/apriltagC]
 * /isApriltag/Center/Y [common_tello_application/apriltagC]
 * /isApriltag/N [common_tello_application/apriltagN]
 * /tello/camera/camera_info [sensor_msgs/CameraInfo]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]

Services: 
 * /camera_apriltag_center/get_loggers
 * /camera_apriltag_center/set_logger_level


contacting node http://192.168.10.4:39107/ ...
Pid: 8126
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /tello/odom
    * to: /tello/tello_driver_node (http://192.168.10.4:36965/)
    * direction: inbound
    * transport: TCPROS
 * topic: /isApriltag
    * to: /camera_apriltag_detection (http://192.168.10.4:46621/)
    * direction: inbound
    * transport: TCPROS
 * topic: /isApriltag/N
    * to: /camera_apriltag_detection (http://192.168.10.4:46621/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/imu
    * to: /tello/tello_driver_node (http://192.168.10.4:36965/)
    * direction: inbound
    * transport: TCPROS
 * topic: /isApriltag/Center/X
    * to: /camera_apriltag_detection (http://192.168.10.4:46621/)
    * direction: inbound
    * transport: TCPROS
 * topic: /isApriltag/Center/Y
    * to: /camera_apriltag_detection (http://192.168.10.4:46621/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/status
    * to: /tello/tello_driver_node (http://192.168.10.4:36965/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/camera/camera_info
    * to: /tello/tello_driver_node (http://192.168.10.4:36965/)
    * direction: inbound
    * transport: TCPROS
```

## [6] camera_apriltag_tracking.py
Tracking the detected apriltag; keep the apriltag on center of image.
Applying PID controller.

1. roslaunch tello_driver tello_node.launch

 ```
Node [/tello/tello_driver_node]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tello/camera/camera_info [sensor_msgs/CameraInfo]
 * /tello/image_raw/h264 [sensor_msgs/CompressedImage]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]
 * /tello/tello_driver_node/parameter_descriptions [dynamic_reconfigure/ConfigDescription]
 * /tello/tello_driver_node/parameter_updates [dynamic_reconfigure/Config]

Subscriptions: 
 * /tello/cmd_vel [geometry_msgs/Twist]
 * /tello/emergency [unknown type]
 * /tello/fast_mode [unknown type]
 * /tello/flattrim [unknown type]
 * /tello/flip [unknown type]
 * /tello/land [std_msgs/Empty]
 * /tello/manual_takeoff [unknown type]
 * /tello/palm_land [unknown type]
 * /tello/takeoff [std_msgs/Empty]
 * /tello/throw_takeoff [unknown type]
 * /tello/video_mode [unknown type]

Services: 
 * /tello/tello_driver_node/get_loggers
 * /tello/tello_driver_node/set_logger_level
 * /tello/tello_driver_node/set_parameters


contacting node http://192.168.10.4:44357/ ...
Pid: 22495
Connections:
 * topic: /tello/image_raw/h264
    * to: /tello/image_compressed
    * direction: outbound
    * transport: TCPROS
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /tello/land
    * to: /tello_teleop (http://192.168.10.4:46683/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/takeoff
    * to: /tello_teleop (http://192.168.10.4:46683/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/cmd_vel
    * to: /tello_teleop (http://192.168.10.4:46683/)
    * direction: inbound
    * transport: TCPROS
 ```
 
2. rosrun common_tello_application camera_apriltag_takeoff_land.py

```
Node [/camera_apriltag_takeoff_land]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tello/land [std_msgs/Empty]
 * /tello/takeoff [std_msgs/Empty]

Subscriptions: 
 * /isApriltag [std_msgs/Bool]
 * /isApriltag/N [common_tello_application/apriltagN]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]

Services: 
 * /camera_apriltag_takeoff_land/get_loggers
 * /camera_apriltag_takeoff_land/set_logger_level


contacting node http://192.168.10.4:34283/ ...
Pid: 3465
Connections:
 * topic: /tello/land
    * to: /tello/tello_driver_node
    * direction: outbound
    * transport: TCPROS
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /tello/takeoff
    * to: /tello/tello_driver_node
    * direction: outbound
    * transport: TCPROS
 * topic: /tello/odom
    * to: /tello/tello_driver_node (http://192.168.10.4:33715/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/status
    * to: /tello/tello_driver_node (http://192.168.10.4:33715/)
    * direction: inbound
    * transport: TCPROS
 * topic: /isApriltag
    * to: /camera_apriltag_detection (http://192.168.10.4:38191/)
    * direction: inbound
    * transport: TCPROS
 * topic: /isApriltag/N
    * to: /camera_apriltag_detection (http://192.168.10.4:38191/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/imu
    * to: /tello/tello_driver_node (http://192.168.10.4:33715/)
    * direction: inbound
    * transport: TCPROS
```

3. rosrun common_tello_application camera_apriltag_center.py

```
Node [/camera_apriltag_center]
Publications: 
 * /isApriltag/objCoord [common_tello_application/objCenter]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /isApriltag [std_msgs/Bool]
 * /isApriltag/Center/X [common_tello_application/apriltagC]
 * /isApriltag/Center/Y [common_tello_application/apriltagC]
 * /isApriltag/N [common_tello_application/apriltagN]
 * /tello/camera/camera_info [sensor_msgs/CameraInfo]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]

Services: 
 * /camera_apriltag_center/get_loggers
 * /camera_apriltag_center/set_logger_level


contacting node http://192.168.10.4:39107/ ...
Pid: 8126
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /tello/odom
    * to: /tello/tello_driver_node (http://192.168.10.4:36965/)
    * direction: inbound
    * transport: TCPROS
 * topic: /isApriltag
    * to: /camera_apriltag_detection (http://192.168.10.4:46621/)
    * direction: inbound
    * transport: TCPROS
 * topic: /isApriltag/N
    * to: /camera_apriltag_detection (http://192.168.10.4:46621/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/imu
    * to: /tello/tello_driver_node (http://192.168.10.4:36965/)
    * direction: inbound
    * transport: TCPROS
 * topic: /isApriltag/Center/X
    * to: /camera_apriltag_detection (http://192.168.10.4:46621/)
    * direction: inbound
    * transport: TCPROS
 * topic: /isApriltag/Center/Y
    * to: /camera_apriltag_detection (http://192.168.10.4:46621/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/status
    * to: /tello/tello_driver_node (http://192.168.10.4:36965/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/camera/camera_info
    * to: /tello/tello_driver_node (http://192.168.10.4:36965/)
    * direction: inbound
    * transport: TCPROS
```

4. rosrun common_tello_application camera_apriltag_tracking.py

```
Node [/camera_apriltag_tracking]
Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tello/cmd_vel [geometry_msgs/Twist]

Subscriptions: 
 * /isApriltag [std_msgs/Bool]
 * /isApriltag/N [common_tello_application/apriltagN]
 * /isApriltag/objCoord [common_tello_application/objCenter]
 * /tello/camera/camera_info [sensor_msgs/CameraInfo]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]

Services: 
 * /camera_apriltag_tracking/get_loggers
 * /camera_apriltag_tracking/set_logger_level


contacting node http://192.168.10.4:33737/ ...
Pid: 16997
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /tello/cmd_vel
    * to: /tello/tello_driver_node
    * direction: outbound
    * transport: TCPROS
 * topic: /isApriltag/objCoord
    * to: /camera_apriltag_center (http://192.168.10.4:33579/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/odom
    * to: /tello/tello_driver_node (http://192.168.10.4:33027/)
    * direction: inbound
    * transport: TCPROS
 * topic: /isApriltag
    * to: /camera_apriltag_detection (http://192.168.10.4:45625/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/imu
    * to: /tello/tello_driver_node (http://192.168.10.4:33027/)
    * direction: inbound
    * transport: TCPROS
 * topic: /isApriltag/N
    * to: /camera_apriltag_detection (http://192.168.10.4:45625/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/status
    * to: /tello/tello_driver_node (http://192.168.10.4:33027/)
    * direction: inbound
    * transport: TCPROS
 * topic: /tello/camera/camera_info
    * to: /tello/tello_driver_node (http://192.168.10.4:33027/)
    * direction: inbound
    * transport: TCPROS
```

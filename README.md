# common_tello_application

## About
A project to understand and controlling the **tello** with **ROS** and **Python**

## Project structure
```
├── CMakeLists.txt
├── etc
│   ├── AprilTag3
│   │   ├── 0.svg
│   │   ├── 10.svg
│   │   ├── 11.svg
│   │   ├── 12.svg
│   │   ├── 13.svg
│   │   ├── 14.svg
│   │   ├── 15.svg
│   │   ├── 16.svg
│   │   ├── 17.svg
│   │   ├── 18.svg
│   │   ├── 19.svg
│   │   ├── 1.svg
│   │   ├── 20.svg
│   │   ├── 2.svg
│   │   ├── 3.svg
│   │   ├── 4.svg
│   │   ├── 5.svg
│   │   ├── 6.svg
│   │   ├── 7.svg
│   │   ├── 8.svg
│   │   └── 9.svg
│   └── Gates-AprilTag3.pdf
├── include
│   └── common_tello_application
├── launch
│   ├── camera_apriltag_center.launch
│   ├── camera_apriltag_detection.launch
│   ├── camera_apriltag_takeoff_land.launch
│   ├── camera_apriltag_tracking.launch
│   ├── camera_bringup.launch
│   └── camera_preview.launch
├── msg
│   ├── apriltagC.msg
│   ├── apriltagCorner.msg
│   ├── apriltagDistance.msg
│   ├── apriltagH.msg
│   ├── apriltagInfo.msg
│   ├── apriltagN.msg
│   ├── arrayCorner.msg
│   ├── arrayHomo.msg
│   ├── FloatArray.msg
│   ├── FloatListList.msg
│   ├── FloatList.msg
│   └── objCenter.msg
├── package.xml
├── README.md
├── script
│   ├── camera_apriltag_align.py
│   ├── camera_apriltag_center.py
│   ├── camera_apriltag_detection.py
│   ├── camera_apriltag_takeoff_land.py
│   ├── camera_apriltag_tracking_align.py
│   ├── camera_apriltag_tracking_mission.py
│   ├── camera_apriltag_tracking_mission_x.py
│   ├── camera_apriltag_tracking.py
│   ├── camera_preview.py
│   └── tello_teleop_key.py
├── setup.py
└── src
    └── common_tello_application
        ├── __init__.py
        ├── makesimpleprofile.py
        ├── makesimpleprofile.pyc
        ├── objcenter.py
        ├── pid.py
        └── pid.pyc
```

## System Requirement (Setup)
### Hardware
1. [Tested!] PC/Laptop: Ubuntu Xenial Xerus (16.04 LTS) -- ROS Kinetic Kame
2. [Tested!] Raspberry Pi 4 8GB: Ubuntu Bionic Beaver (18.04 LTS) -- ROS Melodic Morenia
3. Tello Drone:
	1. Tello [Tested!] 
	2. Tello Iron Man [Supposedly works also] 
	3. Tello EDU [Tested!] 
	
### Software
1. Ubuntu OS:
	1. Download: 
		1. https://releases.ubuntu.com/16.04/
		2. https://releases.ubuntu.com/18.04/
	2. Installation: 
		1. https://ubuntu.com/tutorials/install-ubuntu-desktop-1604#1-overview
		
2. Robot Operating System (ROS)
	1. Kinetic Kame:
		1. http://wiki.ros.org/kinetic/Installation/Ubuntu
	2. Melodic Morena:
		1. http://wiki.ros.org/melodic/Installation/Ubuntu
		
## Cloning Required Repositories

1. Clone **common_tello_application** package:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/KhairulIzwan/common_tello_application.git
$ cd ~/catkin_ws && rosdep install -y --from-paths src --ignore-src --rosdistro kinetic && catkin_make && rospack profile
$ source ~/.bashrc
```
2. Clone **tello_driver** package:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/KhairulIzwan/tello_driver.git
$ cd ~/catkin_ws && rosdep install -y --from-paths src --ignore-src --rosdistro kinetic && catkin_make && rospack profile
$ source ~/.bashrc
```
3. PIP:
```
$ cd ~
$ wget https://bootstrap.pypa.io/get-pip.py
$ sudo python get-pip.py
```
4. AprilTag:
```
$ python -m pip install apriltag
```
5. Imutils:
```
$ python -m pip install imutils
```
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

## AprilTag3 Marking
_Please print out the apriltag marking provided inside the **etc/AprilTag3/** file name **Gates-AprilTag3.pdf**_

## Scripts/Nodes
### tello_teleop_key.py
- [x] Controlling the tello drone using keyboard

#### Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tello/cmd_vel [geometry_msgs/Twist]
 * /tello/land [std_msgs/Empty]
 * /tello/takeoff [std_msgs/Empty]

#### Subscriptions: 
 * None

### camera_preview.py
- [x] Preview an image stream from tello

#### Publications: 
 * /rosout [rosgraph_msgs/Log]

#### Subscriptions: 
 * /tello/image_raw/compressed [sensor_msgs/CompressedImage]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]

#### Services: 
 * /camera_preview/get_loggers
 * /camera_preview/set_logger_level

### camera_apriltag_detection.py
- [x] Detect an AprilTag3 to get useful information

#### Publications: 
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
 * /isApriltag/Distance [common_tello_application/apriltagDistance]
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

#### Subscriptions: 
 * /tello/image_raw/compressed [sensor_msgs/CompressedImage]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]

#### Services: 
 * /camera_apriltag_detection/get_loggers
 * /camera_apriltag_detection/set_logger_level

### camera_apriltag_takeoff_land.py
- [x] Autonomously takeoff and land based on apriltag 0: Land, 1: Takeoff

#### Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tello/land [std_msgs/Empty]
 * /tello/takeoff [std_msgs/Empty]

#### Subscriptions: 
 * /isApriltag [std_msgs/Bool]
 * /isApriltag/N [common_tello_application/apriltagN]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]

#### Services: 
 * /camera_apriltag_takeoff_land/get_loggers
 * /camera_apriltag_takeoff_land/set_logger_level

### camera_apriltag_center.py
- [x] Detect, recognize apriltag and publish the center

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

### camera_apriltag_tracking.py
- [x] Tracking the detected apriltag; keep the apriltag on center of image. Applying PID controller.

### Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tello/cmd_vel [geometry_msgs/Twist]

### Subscriptions: 
 * /isApriltag [std_msgs/Bool]
 * /isApriltag/N [common_tello_application/apriltagN]
 * /isApriltag/objCoord [common_tello_application/objCenter]
 * /tello/camera/camera_info [sensor_msgs/CameraInfo]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]

### Services: 
 * /camera_apriltag_tracking/get_loggers
 * /camera_apriltag_tracking/set_logger_level

### camera_apriltag_tracking_mission.py
- [x] Tracking the detected apriltag; keep the apriltag on center of image and come towards the apriltag. Applying PID controller.

#### Publications: 
 * /rosout [rosgraph_msgs/Log]
 * /tello/cmd_vel [geometry_msgs/Twist]
 * /tello/flip [std_msgs/UInt8]
 * /tello/land [std_msgs/Empty]

#### Subscriptions: 
 * /isApriltag [std_msgs/Bool]
 * /isApriltag/Distance [common_tello_application/apriltagDistance]
 * /isApriltag/Homography/Mat [unknown type]
 * /isApriltag/N [common_tello_application/apriltagN]
 * /isApriltag/objCoord [common_tello_application/objCenter]
 * /tello/camera/camera_info [sensor_msgs/CameraInfo]
 * /tello/imu [sensor_msgs/Imu]
 * /tello/odom [nav_msgs/Odometry]
 * /tello/status [tello_driver/TelloStatus]

#### Services: 
 * /camera_apriltag_tracking/get_loggers
 * /camera_apriltag_tracking/set_logger_level

 
## Mode of Operation

Before proceed:
1. Install Terminator
In Ubuntu, open terminal (Ctrl + Alt + t) and write the following commands:
```
$ sudo add-apt-repository ppa:gnome-terminator
$ sudo apt-get update
$ sudo apt-get install terminator
```
2. Edit bashrc file
	1. Open/Edit file:
	```
	$ gedit ~/.bashrc
	```
	2. Add following at the end of the file
	```
	# Set ROS Kinetic
	source /opt/ros/kinetic/setup.bash
	source ~/catkin_ws/devel/setup.bash

	# Set ROS Network (Tello)
	export ROS_HOSTNAME=192.168.10.4
	export ROS_MASTER_URI=http://192.168.10.4:11311

	# Set ROS alias command
	alias cw='cd ~/catkin_ws'
	alias cs='cd ~/catkin_ws/src'
	alias cm='cd ~/catkin_ws && rosdep install -y --from-paths src --ignore-src --rosdistro kinetic && catkin_make && rospack profile'

	alias eb='gedit ~/.bashrc'
	alias sb='source ~/.bashrc'

	alias gs='git status'
	alias gp='git pull'
	alias ga='git add .'
	NOW=$(date +"%m-%d-%Y-%T")
	alias gc='git commit -m "Updated on $NOW"'
	alias gpu='git push'
	alias gu='gp && git add . && git commit -m "Updated on $NOW" && git push origin main'
	#alias gu='gp && ga && gc && gpu'

	# Set ROS Editor
	export EDITOR='gedit -w'
	```
	3. Save/Close bashrc file and source it again.
	```
	$ source ~/.bashrc
	```
	
### Manual Operation [Tele-Operation]

```
[Teminal: One]
$ roslaunch tello_driver tello_node.launch
[Teminal: Two]
$ rosrun common_tello_application camera_preview.py
[Teminal: Three]
$ rosrun common_tello_application teleop_key.py
```

### Autonomous Operation
#### AprilTag3 Detection
```
[Teminal: One]
$ roslaunch tello_driver tello_node.launch
[Teminal: Two]
$ rosrun common_tello_application camera_apriltag_detection.py
```

#### AprilTag3 Takeoff and Land
```
[Teminal: One]
$ roslaunch tello_driver tello_node.launch
[Teminal: Two]
$ rosrun common_tello_application camera_apriltag_detection.py
[Teminal: Three]
$ rosrun common_tello_application camera_apriltag_takeoff_land.py
```

#### AprilTag3 Tracking
```
[Teminal: One]
$ roslaunch tello_driver tello_node.launch
[Teminal: Two]
$ rosrun common_tello_application camera_apriltag_detection.py
[Teminal: Three]
$ rosrun common_tello_application camera_apriltag_takeoff_land.py
[Teminal: Four]
$ rosrun common_tello_application camera_apriltag_center.py
[Teminal: Five]
$ rosrun common_tello_application camera_apriltag_tracking.py
```

#### AprilTag3 Tracking Mission
```
[Teminal: One]
$ roslaunch tello_driver tello_node.launch
[Teminal: Two]
$ rosrun common_tello_application camera_apriltag_detection.py
[Teminal: Three]
$ rosrun common_tello_application camera_apriltag_takeoff_land.py
[Teminal: Four]
$ rosrun common_tello_application camera_apriltag_center.py
[Teminal: Five]
$ rosrun common_tello_application camera_apriltag_tracking_mission.py
```

## Reference:
1. Pan/tilt face tracking with a Raspberry Pi and OpenCV
	1. https://www.pyimagesearch.com/2019/04/01/pan-tilt-face-tracking-with-a-raspberry-pi-and-opencv/
2. Find distance from camera to object/marker using Python and OpenCV
	1. https://www.pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
3. Measuring size of objects in an image with OpenCV
	1. https://www.pyimagesearch.com/2016/03/28/measuring-size-of-objects-in-an-image-with-opencv/
4. Measuring distance between objects in an image with OpenCV
	1. https://www.pyimagesearch.com/2016/04/04/measuring-distance-between-objects-in-an-image-with-opencv/

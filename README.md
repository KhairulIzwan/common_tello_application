# common_tello_application

```
common_tello_application
├── CMakeLists.txt
├── include
│   └── common_tello_application
├── launch
│   ├── camera_apriltag_center.launch
│   ├── camera_apriltag_recog.launch
│   ├── camera_apriltag_takeoff_land.launch
│   ├── camera_apriltag_tracking.launch
│   ├── camera_bringup.launch
│   └── camera_preview.launch
├── msg
│   └── objCenter.msg
├── package.xml
├── README.md
├── script
│   ├── camera_apriltag_center.py
│   ├── camera_apriltag.py
│   ├── camera_apriltag_takeoff_land.py
│   ├── camera_apriltag_tracking.py
│   ├── camera_converter.py
│   ├── camera_preview.py
│   ├── camera_resize.py
│   └── tello_teleop_key.py
├── setup.py
└── src
    └── common_tello_application
        ├── __init__.py
        ├── makesimpleprofile.py
        ├── objcenter.py
        └── pid.py

```

# About
A project to understand and controlling the **tello**

# Required Packages/Library
1. Tello Driver:
	1. http://wiki.ros.org/tello_driver
	2. git clone https://github.com/appie-17/tello_driver

2. AprilTag:
	1. https://pypi.org/project/apriltag/
	2. python -m pip install apriltag

3. Imutils:
	1. https://pypi.org/project/imutils/
	2. python -m pip install pip install imutils

# How it works?
## [1] tello_teleop_key.py
Controlling the tello drone using keyboard

1. roslaunch tello_driver tello_node.launch
2. rosrun common_tello_application tello_teleop_key.py

## [2] camera_resize.py
Convert the image into smaller size (for viewing purpose)

1. roslaunch tello_driver tello_node.launch
2. rosrun common_tello_application camera_resize.py

## [3] camera_converter.py
Convert the image into smaller size (for viewing purpose)

1. roslaunch tello_driver tello_node.launch
2. rosrun common_tello_application camera_converter.py

## [4] camera_preview.py
Preview an image stream from tello

1. roslaunch tello_driver tello_node.launch
2. rosrun common_tello_application camera_converter.py
3. rosrun common_tello_application camera_preview.py

## [5] camera_apriltag.py
Detect and recognize apriltag

1. roslaunch tello_driver tello_node.launch
2. rosrun common_tello_application camera_converter.py
3. rosrun common_tello_application camera_apriltag.py

## [6] camera_apriltag_center.py
Detect, recognize apriltag and publish the center

1. roslaunch tello_driver tello_node.launch
2. rosrun common_tello_application camera_apriltag_center.py

## [7] camera_apriltag_takeoff_land.py
Autonomously takeoff and land based on apriltag 0: Land, 1: Takeoff

1. roslaunch tello_driver tello_node.launch
2. rosrun common_tello_application camera_apriltag_takeoff_land.py

## [8] camera_apriltag_tracking.py
Tracking the detected apriltag; keep the apriltag on center of image.
Applying PID controller.

1. roslaunch tello_driver tello_node.launch
2. rosrun common_tello_application camera_apriltag_center.py
3. rosrun common_tello_application camera_apriltag_tracking.py

# common_tello_application

```
common_tello_application
│
├── CMakeLists.txt
├── include
│   └── common_tello_application
├── launch
│   ├── camera_apriltag_center.launch
│   ├── camera_apriltag_recog.launch
│   ├── camera_apriltag_takeoff_land.launch
│   └── camera_preview.launch
├── package.xml
├── README.md
├── script
│   ├── camera_apriltag_center.py [6]
│   ├── camera_apriltag.py [5] 
│   ├── camera_apriltag_takeoff_land.py
│   ├── camera_apriltag_tracking.py
│   ├── camera_converter.py [3] 
│   ├── camera_preview.py [4] 
│   ├── camera_resize.py [2] 
│   └── tello_teleop_key.py [1] 
├── setup.py
└── src
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

**operation**
1. roslaunch tello_driver tello_node.launch
2. rosrun common_tello_application tello_teleop_key.py

## [2] camera_resize.py
Convert the image into smaller size (for viewing purpose)

**operation**
1. roslaunch tello_driver tello_node.launch
2. rosrun common_tello_application camera_resize.py

## [3] camera_converter.py
Convert the image into smaller size (for viewing purpose)

**operation**
1. roslaunch tello_driver tello_node.launch
2. rosrun common_tello_application camera_converter.py

## [4] camera_preview.py
Preview an image stream from tello

**operation**
1. roslaunch tello_driver tello_node.launch
2. rosrun common_tello_application camera_converter.py
3. rosrun common_tello_application camera_preview.py

*or*
2. roslaunch common_tello_application camera_preview.launch

## [5] camera_apriltag.py
Detect and recognize apriltag

**operation**
1. roslaunch tello_driver tello_node.launch
2. rosrun common_tello_application camera_converter.py
3. rosrun common_tello_application camera_apriltag.py

*or*
2. roslaunch common_tello_application camera_apriltag_recog.launch

## [6] camera_apriltag_center.py
Detect, recognize apriltag and publish the center

**operation**
1. roslaunch tello_driver tello_node.launch
2. rosrun common_tello_application camera_apriltag_center.py

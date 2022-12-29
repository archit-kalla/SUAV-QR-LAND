# SUAV-QR-LAND

A project worked on along side the Solar UAV Lab at the University of Minnesota.

## Goals

Every vehicle capable of flight has two sequences that are of the utmost importance: takeoff and landing. Without takeoff, a UAV would never achieve autonomous flight, and without landing, it would never fly again. Both are critical procedures for any UAV that seeks to reach commercial viability. This project seeks to focus on the latter, more valuable of the two algorithms. 
By providing a safe, efficient landing procedure with an emphasis on low power consumption, we can be assured that our UAV will be able to land in both regular use situations and emergency low power situations. This heavily reduces the cost of operation, as a safe landing ensures there is a flight capable drone ready for recovery and further missions.
To begin our project, we knew that we would need to have some form of understanding the environment a drone is in. We discussed our options, from attaching a LiDAR module to the UAV to mounting a camera setup onto the UAV. We settled on using a stereo camera module as that would fulfill the following conditions:
- Considerably lightweight, weighing less than 100 grams
- Power-efficient, using less than 10 watts of power
- Be able to provide suitable depth information for UAV landing 

## Overview of code

This folder contains the necessary files needed to achieve movement to a QR landing pad. The files descriptions in this folder are as follows:

 - QR_Landing.py: This file contains the main ROS function that attempts to land the drone on QR code. **The file here differs from what is present on the Pi as we were unable to copy the file from the Pi and put it in the folder, so the Pi has the most up-to-date “working” version of the landing code. The code should be in ~/catkin_ws/src/QR_LAND/scripts/QR_Landing.py.** This is written in python 2.7. The code leverages pyzbar to identify the qr code on in the image. Then using the bounding box from this package we find a center point and determine how far in x and y coordinates the qr code is in terms of the center of the camera. We then move the drone to those coordinates. Important to note that these coordinates that are published are absolute coordinates in the real world, not relative to the drone. **We made an oversight in the code that needs to be corrected for proper landing: when publishing the coordinates we forgot to add the qr relative position to the drones current position. The fix for this is when publishing qr_pos do the following:**

```python
#publish position
last_xy = ( feedback_pos.pose.position.x + qr_pos[0], feedback_pos.pose.position.y + qr_pos[1])
pose.pose.position.x = last_xy[0]
pose.pose.position.y = last_xy[1]
```

This should fix the issue where the drone would land incorrectly. 
If a QR code is not detected, the drone should move in a spiral formation; the code for this is present on the Pi version. We recommend referring to that instead of the one in this folder

 - Examine_area.py: An empty file, we were unable to implement traversing the space in an effective manner, QR_landing.py has a temporary solution to just move in a spiral to land on the Pi. In reality we would want this file to include code to handle cases where partial qr detection occurs, right now that is not handled in the pyzbar package.

 - Test_qr_detection.py: self explanatory, tests for qr code in the frame of the camera and displays bounding boxes.

 - Get_calib_images.py:  Gets calibration images for undistortion of the camera for more precise movements. We were unable to undistort so this is not needed for QR_land to function. Only needs to be done once

 - Get_camera_distortion.py:  gets the distortion matrix and saves it as a .npz file. This file is then used in the undistort() function in qr_landing.py to undistort the images, but we chose not to undistort the image as it cost performance. If you choose to use this the B.npz must be present in the same folder as QR_landing in the ros folder ~/catkin_ws/src/QR_LAND/scripts/ so that the undistort() function works properly. This file only needs to be run once to get the camera lens distortion.

To run the code on the drone simply input this into the ssh terminal on the drone “rosrun QR_LAND QR_landing.py”.

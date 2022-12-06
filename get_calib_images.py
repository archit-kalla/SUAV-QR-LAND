# take calibration images

import cv2
import numpy as np
import time
import os
import PiCamera

#set the camera
camera = PiCamera()
camera.resolution = (1920, 1080)
camera.framerate = 30
camera.start_preview()
time.sleep(2)



#set the number of images to take
num_images = 30

#set the delay between images
delay = 5

#set the folder to save the images
folder = 'calib_images'

#make the folder if it doesn't exist
if not os.path.exists(folder):
    os.makedirs(folder)

#take the images
for i in range(num_images):
    camera.start_preview()
    time.sleep(2)
    #get the image
    img = camera.capture('calib_images/image%02d.jpg' % i)



#release the camera



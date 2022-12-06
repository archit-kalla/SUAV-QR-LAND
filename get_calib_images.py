# take calibration images

import cv2
import numpy as np
import time
import os

#set the camera
cap = cv2.VideoCapture(0)

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
    #get the image
    ret, frame = cap.read()
    #show the image
    cv2.imshow('frame', frame)
    cv2.waitKey(1)
    cv2.destroyAllWindows()
    #save the image
    cv2.imwrite(folder + '/calib_' + str(i) + '.png', frame)
    #wait for the delay
    time.sleep(delay)

#release the camera
cap.release()


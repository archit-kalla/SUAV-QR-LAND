import pyzbar
from pyzbar.pyzbar import ZBarSymbol

import cv2

import numpy as np

import math

from picamera import PiCamera

camera = PiCamera()
camera.resolution = (1280, 720)
camera.framerate = 15
mtx, dist = None, None
def read_kd():
    with np.load('B.npz') as X:
        mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
    return mtx, dist
mtx,dist = read_kd()
while(1):
    image = np.empty((camera.resolution[1] * camera.resolution[0] * 3,), dtype=np.uint8)
    camera.capture(image, 'bgr')
    new_img = image.reshape((camera.resolution[1], camera.resolution[0], 3))
    gray = cv2.cvtColor(new_img, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape[:2]
    K=mtx
    D=dist
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    img = cv2.undistort(gray, K, D, None, newcameramtx)
    frame =0
    counter = 0
    cv2.undistort(img, mtx, dist, None, mtx)
    try:
        qr = pyzbar.pyzbar.decode(img, symbols=[ZBarSymbol.QRCODE])
        bbox = qr[0].rect
        center = (bbox.left + int(bbox.width/2), bbox.top + int(bbox.height/2))

        #display the bounding box
        cv2.rectangle(img, (bbox.left, bbox.top), (bbox.left + bbox.width, bbox.top + bbox.height), (0, 0, 255), 2)
        cv2.imshow('img', img)
    except:
        print("No QR code found Frame:"+str(frame) +" Count: "+ str(counter))
        cv2.imshow('img', img)
        counter+=1
        continue
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break
    

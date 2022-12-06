import pyzbar
from pyzbar.pyzbar import ZBarSymbol

import cv2

import numpy as np

import math

cap = cv2.VideoCapture(0)
while(1):
    _,img = cap.read()
    cv2.imshow('img',img)
    try:
        qr = pyzbar.pyzbar.decode(img, symbols=[ZBarSymbol.QRCODE])
        bbox = qr[0].rect
        center = (bbox.left + int(bbox.width/2), bbox.top + int(bbox.height/2))

        #display the bounding box
        cv2.rectangle(img, (bbox.left, bbox.top), (bbox.left + bbox.width, bbox.top + bbox.height), (0, 0, 255), 2)
        cv2.imshow('img', img)
    except:
        print("No QR code found")
        continue
    if cv2.waitKey(25) & 0xFF == ord('q'):
        break
    

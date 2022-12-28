#!/usr/bin/python

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
# added CommandTOL services
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest, CommandTOL, CommandTOLRequest
import cv2
import pyzbar
from pyzbar.pyzbar import ZBarSymbol
import random
from picamera import PiCamera

#from pyquaternion import Quaternion



current_state = State()
land_cmd = None
offb_set_mode = None
arm_cmd = None
land_cmd = None
init_pos = (0,0,0)
takeoff_height = 1.5
#initialize camera
# cap = cv2.VideoCapture(0)
camera = PiCamera()
camera.resolution = (1280, 720)
camera.framerate = 15
#q = Quaternion(axis=[0, 0, 1], angle=math.radians(90))

#q = q.yaw_pitch_roll[0]


mtx, dist = None, None
#read k and d from camera.npz
def read_kd():
    with np.load('B.npz') as X:
        mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
    return mtx, dist

def state_cb(msg):
    global current_state
    current_state = msg
    
def feedback_cb(geometry_msgs):
    global feedback_pos
    feedback_pos = geometry_msgs

#undisotrt the image
def undistort(img):
    #get the camera matrix
    K = mtx #need to get this from the camera
    #get the distortion coefficients
    D = dist #need to get this from the camera

    #undistort the image
    h, w = img.shape[:2]
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
    dst = cv2.undistort(img, K, D, None, newcameramtx)

    #crop the image
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    return dst

def is_qr(img):
    #use pyzbar to find the qr code in the image and bounding box
    #find the qr code in the image
    qr = pyzbar.pyzbar.decode(img, symbols=[ZBarSymbol.QRCODE])

    #if there is a qr code in the image
    if len(qr) > 0:
        #get the bounding box of the qr code
        bbox = qr[0].rect

        #return the bounding box of the qr code
        return True, bbox
    return False, None


#take a point in an image and return the realworld coordinates of that point
def center_bbox(bbox):
    #get the center of the qr code
    center = (bbox.left + int(bbox.width/2), bbox.top + int(bbox.height/2))

    #return the center of the qr code
    return center


def get_qr_pos(img,center_bbox):
    #get the real world coordinates of the center of the qr code 
    FOV_Y = 32.4
    FOV_X = 55
    QR_SIZE = 0.4
    CAMERA_HEIGHT = 1.5


    #get the center of the image
    h, w = img.shape[:2]
    center_img = (int(w/2), int(h/2))

    #get the distance from the center of the image to the center of the qr code
    dist_x = center_img[0] - center_bbox[0]
    dist_y = center_img[1] - center_bbox[1]

    #get the angle from the center of the image to the center of the qr code
    angle_x = math.radians(FOV_X*(float(dist_x)/float(w)))
    angle_y = math.radians(FOV_Y*float(dist_y)/float(w))

    #get the distance from the camera to the qr code
    dist = CAMERA_HEIGHT/math.cos(angle_y)

    #get the real world coordinates of the center of the qr code
    x = dist*math.sin(angle_x)
    y = dist*math.sin(angle_y)

    #return the real world coordinates of the center of the qr code
    return (x,y)
#calculate the distance between 2 points
def dist(x1,y1,x2,y2):
    dist = math.sqrt((x1-x2)**2 + (y1-y2)**2)


    
if __name__ == "__main__":
    rospy.init_node("offb_landing_py")
    state_sub = rospy.Subscriber("mavros/state", State, callback=state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
   	
    pos_feedback = rospy.Subscriber("mavros/local_position/pose", PoseStamped, feedback_cb)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rospy.wait_for_service("/mavros/cmd/land")
    land_client = rospy.ServiceProxy("mavros/cmd/land", CommandTOL)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = init_pos[0]		
    pose.pose.position.y = init_pos[1]	
    pose.pose.position.z = init_pos[2]
    #make pose.pose.orientation forward left up 
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0.707
    pose.pose.orientation.w = 0.707


    # Send a few setpoints before starting
    # for i in range(100):   
    #     if(rospy.is_shutdown()):
    #         break

    #     local_pos_pub.publish(pose)
    #     rate.sleep()
    # for i in range(100):
    #     if(rospy.is_shutdown()):
    #         break

    #     local_pos_pub.publish(pose)
    #     rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    land_cmd= CommandTOLRequest()
    
    last_req = rospy.Time.now()
    last_xy = (init_pos[0], init_pos[1])
    image = np.empty((camera.resolution[1] * camera.resolution[0] * 3,), dtype=np.uint8)

    land_bool = False
    test_qr = False

    while current_state.mode!= "OFFBOARD":
        print("waitin for offboard")
        local_pos_pub.publish(pose)
        rate.sleep()

    while not rospy.is_shutdown():
        local_pos_pub.publish(pose)
        if land_bool:
            print("Landing")
            land_client.call(land_cmd)
            #need to break once done 
            if feedback_pos.pose.position.z < 0.1:
                print("landed")
                break
            rate.sleep()
            continue
        #publish if not sufficent time has passed since last movement request
        if (rospy.Time.now() - last_req) < rospy.Duration(3.0):
            print("publishing last request")
            pose.pose.position.x = last_xy[0]
            pose.pose.position.y = last_xy[1]
            local_pos_pub.publish(pose)
            rate.sleep()
            continue
        if test_qr:
            if (abs(feedback_pos.pose.position.x - last_xy[0]) < 0.1 and abs(feedback_pos.pose.position.y - last_xy[1])) < 0.1:
                print("Landing")
                land_client.call(land_cmd)
                land_bool = True
                rate.sleep()
                continue

        if current_state.mode != "OFFBOARD":
            print("waiting until offboard")
            #last_req = rospy.Time.now()
            rate.sleep()
            continue
        
        # if not current_state.armed:
        #     #last_req = rospy.Time.now()
        #     print("waiting until armed")
        #     arming_client.call(arm_cmd)
        #     rate.sleep()
        #     continue
        camera.capture(image, 'bgr')
        img = image.reshape((camera.resolution[1], camera.resolution[0], 3))
        if img.any()!=None:
            #test for qr code
            test_qr, bbox = is_qr(img)
            if test_qr:
                print("qr code detected")
                #get the center of the qr code
                center_point = center_bbox(bbox)
                qr_pos = get_qr_pos(img,center_point)
                #publish position
                last_xy = qr_pos
                pose.pose.position.x = qr_pos[0]
                pose.pose.position.y = qr_pos[1]
                print("x: " + str(qr_pos[0]) + " y: " + str(qr_pos[1]))
                local_pos_pub.publish(pose)
                last_req = rospy.Time.now()
            else:
                print("no qr code detected")
                #publish last random position from x =[-.5,.5] y =[-1,.5]
                last_xy = (random.uniform(-.5,.5), random.uniform(-1,.5))
                pose.pose.position.x = last_xy[0]
                pose.pose.position.y = last_xy[1]
                local_pos_pub.publish(pose)
                last_req = rospy.Time.now()

        rate.sleep()
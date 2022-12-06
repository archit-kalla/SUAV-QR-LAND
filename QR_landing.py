#python 2.7

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
    #assume fov of camera is 60 degrees
    #assume the camera is 1 meter above the ground
    #assume the qr code is on the ground
    #assume the qr code is 40cm by 40cm
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
    angle_x = math.radians(FOV_X*(dist_x/w))
    angle_y = math.radians(FOV_Y*(dist_y/h))

    #get the distance from the camera to the qr code
    dist = CAMERA_HEIGHT/math.cos(angle_y)

    #get the real world coordinates of the center of the qr code
    x = dist*math.cos(angle_x)
    y = dist*math.sin(angle_x)

    #return the real world coordinates of the center of the qr code
    return (x,y)
    
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


    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()
    
    #takeoff
    pose.pose.position.z = takeoff_height
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    land_cmd= CommandTOLRequest()
    
    last_req = rospy.Time.now()
    last_xy = (init_pos[0], init_pos[1])



    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
            set_mode_client(offb_set_mode)
            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req > rospy.Duration(5.0)):
                arming_client(arm_cmd)
                last_req = rospy.Time.now()
            else:
                if current_state.armed:
                    image = np.empty((camera.resolution[1] * camera.resolution[0] * 3,), dtype=np.uint8)
                    camera.capture(image, 'bgr')
                    img = image.reshape((camera.resolution[1], camera.resolution[0], 3))
                    img = undistort(img)
                    if img:
                        is_qr_test, bbox = is_qr(img)
                        if is_qr_test:
                            center = center_bbox(bbox)
                            x, y = get_qr_pos(center)
                            while (feedback_pos.pose.position.x!= last_xy[0] and feedback_pos.pose.position.y!= last_xy[1]) or not (rospy.Time.now() - last_req > rospy.Duration(5.0)):
                                print("waiting for feedback_1")
                                local_pos_pub.publish(pose)
                                rate.sleep()
                            if (x,y)<=(0.05,0.05) and (x,y)>=(-0.05,-0.05): #tolerance
                                print("landing")
                                land_client(land_cmd)
                                break
                            else:
                                print("moving to qr code")
                                pose.pose.position.x = feedback_pos.pose.position.x + x
                                pose.pose.position.y = feedback_pos.pose.position.y + y
                                last_xy = (feedback_pos.pose.position.x + x,feedback_pos.pose.position.y+y)
                                local_pos_pub.publish(pose)
                                rate.sleep()
                        else:
                            print("No QR code detected")
                            print("moving to random position")
                            #wait until feedback_pos is updated
                            while (feedback_pos.pose.position.x!= last_xy[0] and feedback_pos.pose.position.y!= last_xy[1]) or (rospy.Time.now() - last_req > rospy.Duration(5.0)):
                                print("waiting to update feedback_pos")
                                local_pos_pub.publish(pose)
                                rate.sleep()
                            pose.pose.position.x = random.uniform(-2,2)
                            pose.pose.position.y = random.uniform(-2,2)
                            print("x: ", pose.pose.position.x)
                            print("y: ", pose.pose.position.y)
                            last_xy = (pose.pose.position.x, pose.pose.position.y)
                            last_req = rospy.Time.now()
                            local_pos_pub.publish(pose)
                            rate.sleep()
                    else:
                        print("No image detected")
                        rate.sleep()
                else:
                    arming_client(arm_cmd)
                    last_req = rospy.Time.now()
                    rate.sleep()

#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge 
import imutils
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
# from math import pi
# from std_msgs.msg import Float32

'''
Function for detecting the ball 
and returning its coordinate
'''

dist = lambda x1,y1,x2,y2: (x1-x2)**2+(y1-y2)**2

def detectBall(frame):
    global counter, X, Y, cX, cY
    counter += 1

    found = False
    ####### HOUGH ######
    img = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    cimg = cv2.medianBlur(img,5)
    chosen = None
    cX = cY = 0
    finalWidth = 0
    circles = cv2.HoughCircles(cimg,cv2.HOUGH_GRADIENT,0.7,100,param1=100,param2=20,minRadius=5,maxRadius=70)
    if circles is not None:
        found = True
        circles = np.uint16(np.around(circles))
        for i in circles[0,:]:
            cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),1) # draw the outer circle
            cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3) # draw the center of the circle
            if chosen is None:
                chosen = i
            else:
                if chosen[2] <= i[2]:
                    chosen = i
            finalWidth = chosen[2] * 2 # THIS IS DIAMETER
            cX = chosen[0]
            cY = chosen [1]
    cv2.imshow('detected circles',cimg)
    cv2.imshow('og',frame)
    cv2.waitKey(1)
    

    if found: 
        return cX, cY, cimg, finalWidth
    else:
        return -100, -100, cimg, 0

def calc_dist(width):
    #(known radius x focal length)/apparent radius
    if width:
        return (5 * 8.72)/ (width)
    return 0


# Callback called whenever image received
def image_callback(data):
    # convert to numpy -> RGB
    image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    _ , w = image.shape[:2]

    # image = imutils.resize(image, width=int(w*8))
    cX, cY, mask, ballWidth = detectBall(image)

    # Create Point instance and set x, y methods
    point = Point()
    point.x = cX
    point.y = cY
    point.z = w 	# width of the frame
    pub_point.publish(point)	# Publish point on the publisher

    angle = -1
    if cX >= 0: 
        angle = ((cX - int(w/2))/w)*1.0856
    point = Point()
    point.x = calc_dist(ballWidth)
    point.z = angle
    pub_dist.publish(point)

    # just displaying it
    length = int(w/100)
    (startX, endX) = (int(cX - length), int(cX + length))
    (startY, endY) = (int(cY - length), int(cY + length))
    cv2.line(image, (startX, cY), (endX, cY), (0, 0, 255), 2)
    cv2.line(image, (cX, startY), (cX, endY), (0, 0, 255), 2)

    image = imutils.resize(image, width=500)
    mask = imutils.resize(mask, width=500)
    # cv2.imshow('image',image)
    # cv2.imshow('mask',mask)
    # cv2.waitKey(1)


def dist_callback(data):
    print("Distance: ", data.x, " angle:", data.z)


if __name__ == '__main__':
    global counter, X, Y, pub_point, pub_dist
    counter = 0
    X, Y = [], []

    # Initialize the node
    rospy.init_node('find_ball', anonymous=True)

    # Subscribe to raspicam_node and receive the image
    img_sub = rospy.Subscriber("/camera/rgb/image_raw/compressed",\
        CompressedImage, image_callback)
    dist_sub = rospy.Subscriber("/ball_dist", Point, dist_callback)
    bridge = CvBridge()

    # Publish x-y coordinates over ball_location topic
    pub_point = rospy.Publisher('/ball_location', Point, queue_size=5)
    pub_dist = rospy.Publisher('/ball_dist', Point, queue_size=5)
    rospy.spin()

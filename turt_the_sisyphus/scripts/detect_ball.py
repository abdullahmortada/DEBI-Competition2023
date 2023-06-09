#!/usr/bin/env python3
import rospy
import cv2
from cv_bridge import CvBridge 
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage
# import imutils
# from math import pi
# from std_msgs.msg import Float32

'''
Function for detecting the ball 
and returning its coordinate
'''


def detectBall(frame):
    global counter, X, Y, cX, cY
    counter += 1

    # lime-green
    colorLower = ( 25, 50,  0)
    colorUpper = ( 90,255,255)
    i1, i2 = 1, 1

    # blue
    # colorLower = ( 90, 70,70)
    # colorUpper = (120,200,255)
    # i1, i2 = 0, 5

    # red
    colorLower = (  0, 50,50)
    colorUpper = ( 35,255,255)
    i1, i2 = 3, 5

    # Convert to HSV color-space and create the mask
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, colorLower, colorUpper)
    mask = cv2.erode(mask, None, iterations=i1)
    mask = cv2.dilate(mask, None, iterations=i2)

    # Find all contours after a series of erosion/dilation
    (cnts, _) = cv2.findContours(mask.copy(), \
        cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    # Initializing x & y lists every nth frame
    if counter%4 == 0:
        X, Y = [], []

    foundBall = False
    finalWidth = 0
    # For each contour, get area, perimeter, filter, and find centroid
    for (i,c) in enumerate(cnts):
        # area = cv2.contourArea(c)
        # perimeter = cv2.arcLength(c, True)
        rect = cv2.minAreaRect(c)
        # if area > 900 and perimeter > 90:
        if rect[1][0] > 20:
            # print ("Contour #%d -- area: %.2f, perimeter: %.2f" \
            #     % (i + 1, area, perimeter))			
            foundBall = True
    if foundBall:
        c = max(cnts, key=cv2.contourArea)
        finalWidth = cv2.minAreaRect(c)[1][0] 
        M = cv2.moments(c)
        (cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        X.append(cX)
        Y.append(cY)

    # Average out each x & y location determined
    if X and Y:
        cX = int(sum(X)/len(X))
        cY = int(sum(Y)/len(Y))
        return cX, cY, mask, finalWidth
    else:
        return -100, -100, mask, 0


def calc_dist(width):
    #(known radius x focal length)/apparent radius
    if width:
        return (38)/ (width)
    return 0


# def calc_dist(diam, img_height):
#     #(known radius x focal length)/apparent radius
#     if diam:
#         return (3.6 * 55 * img_height)/(diam * 2.813 * 1000)
#     return 0


# Callback called whenever image received
def image_callback(data):
    # convert to numpy -> RGB
    image = bridge.compressed_imgmsg_to_cv2(data, "bgr8")
    h , w = image.shape[:2]

    # image = imutils.resize(image, width=int(w*8))
    cX, cY, mask, ballWidth = detectBall(image)

    # Create Point instance and set x, y methods
    point = Point()
    point.x = cX
    point.y = cY
    point.z = w 	# width of the frame
    pub_point.publish(point)	# Publish point on the publisher

    angle = -1
    yAngle = -1
    if cX >= 0: 
        angle = ((cX - int(w/2))/w)*1.0856
        yAngle = ((cY - int(h/2))/h)*0.8517
        print(yAngle)

    if yAngle < 0.32:
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

    # image = imutils.resize(image, width=500)
    # mask = imutils.resize(mask, width=500)
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

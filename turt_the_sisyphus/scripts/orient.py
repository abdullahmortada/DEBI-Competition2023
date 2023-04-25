#!/usr/bin/env python3
import rospy

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

# Callback function called whenever
# x-y coordinate received
def drive_callback(data):
    ball_x 	= data.x
    ball_y 	= data.y
    width  	= data.z

    # Create Twist() instance
    vel = Twist()

    # 
    if ball_x < 0 and ball_y < 0:
        vel.angular.z = 0
    else:
        # Determine center-x, normalized deviation from center
        mid_x  	= int(width/2)
        delta_x	= ball_x - mid_x
        norm_x 	= delta_x/width

        if norm_x > 0.1:
            print ("delX: {:.3f}. Turn right".format(norm_x))
            vel.angular.z = -0.5
        elif norm_x < -0.1:
            print ("delX: {:.3f}. Turn left".format(norm_x))
            vel.angular.z = 0.5
        if abs(norm_x) < 0.1:
            print ("delX: {:.3f}. Stay in center".format(norm_x))
            vel.angular.z = 0
    # publish vel on the publisher
    pub_vel.publish(vel)


def dist_callback(data):
    print("Distance: ", data.data)


if __name__ == '__main__':
    global pub_vel

    # intialize the node
    rospy.init_node('drive_wheel', anonymous=True)

    # subscribe to /ball_location topic to receive coordinates
    img_sub = rospy.Subscriber("/ball_location",Point, drive_callback)
    dist_sub = rospy.Subscriber("/ball_dist", Float32, dist_callback)

    # publish to /cmd_vel topic the angular-z velocity change
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rospy.spin()

#!/usr/bin/env python3
import rospy

from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

# Callback function called whenever
# x-y coordinate received
def drive_callback(data):
    if is_moving:
        return 

    vel = Twist()
    if data.x < 0:
        pass
    else:
        # Determine center-x, normalized deviation from center

        if data.z > 0.09:
            print ("angle: {:.3f}. Turn right".format(data.z))
            vel.angular.z = -0.5
        elif data.z < -0.09:
            print ("angle: {:.3f}. Turn left".format(data.z))
            vel.angular.z = 0.5
        if abs(data.z) < 0.09:
            print ("angle: {:.3f}. Stay in center".format(data.z))
    # publish vel on the publisher
    pub_vel.publish(vel)


def moving_callback(data):
    global is_moving 
    is_moving = data.data


if __name__ == '__main__':
    global pub_vel, is_moving

    is_moving = False

    # intialize the node
    rospy.init_node('orient', anonymous=True)

    # subscribe to /ball_location topic to receive coordinates
    img_sub = rospy.Subscriber("/ball_dist",Point, drive_callback)
    move_sub = rospy.Subscriber("/is_moving",Bool, moving_callback)

    # publish to /cmd_vel topic the angular-z velocity change
    pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rospy.spin()

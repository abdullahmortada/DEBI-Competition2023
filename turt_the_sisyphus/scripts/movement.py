import rospy
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
import tf
from math import sqrt, pow, pi, atan2
from tf.transformations import euler_from_quaternion
import numpy as np


def Callback(data):
    print("starting at ", data)
    def get_odom(tf_listener, odom_frame, base_frame):
        try:
            (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)

        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

    move_cmd = Twist()
    rate = rospy.Rate(20)
    tf_listener = tf.TransformListener()
    odom_frame = 'odom' 
    is_ready.publish(Bool(False))

    try:
        tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
        base_frame = 'base_footprint'
    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        try:
            tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
            base_frame = 'base_link'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
            rospy.signal_shutdown("tf Exception")
            return 

    
    (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)
    last_rotation = 0
    linear_speed = 1
    angular_speed = 1
    (goal_x, goal_y, goal_z) = (data.x, data.y, data.z)
    goal_z = np.deg2rad(goal_z)
    goal_angle = atan2(goal_y - position.y, goal_x - position.x)
    print("goal angle:", goal_angle)
    

    while abs(rotation - goal_angle) > 0.05:
        (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)
        x_start = position.x
        y_start = position.y
        if goal_angle >= 0:
            if rotation <= goal_angle and rotation >= goal_angle - pi:
                move_cmd.linear.x = 0.00
                move_cmd.angular.z = 0.5
            else:
                move_cmd.linear.x = 0.00
                move_cmd.angular.z = -0.5
        else:
            if rotation <= goal_angle + pi and rotation > goal_angle:
                move_cmd.linear.x = 0.00
                move_cmd.angular.z = -0.5
            else:
                move_cmd.linear.x = 0.00
                move_cmd.angular.z = 0.5
        cmd_vel.publish(move_cmd)
        rate.sleep()


    goal_distance = sqrt(pow(goal_x - position.x, 2) + pow(goal_y - position.y, 2))
    distance = goal_distance

    while distance > 0.05:
        (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)
        x_start = position.x
        y_start = position.y
        path_angle = atan2(goal_y - y_start, goal_x- x_start)

        if path_angle < -pi/4 or path_angle > pi/4:
            if goal_y < 0 and y_start < goal_y:
                path_angle = -2*pi + path_angle
            elif goal_y >= 0 and y_start > goal_y:
                path_angle = 2*pi + path_angle
        if last_rotation > pi-0.1 and rotation <= 0:
            rotation = 2*pi + rotation
        elif last_rotation < -pi+0.1 and rotation > 0:
            rotation = -2*pi + rotation
        move_cmd.angular.z = angular_speed * path_angle-rotation

        distance = sqrt(pow((goal_x - x_start), 2) + pow((goal_y - y_start), 2))
        move_cmd.linear.x = min(linear_speed * distance, 0.4)

        if move_cmd.angular.z > 0:
            move_cmd.angular.z = min(move_cmd.angular.z, 1.5)
        else:
            move_cmd.angular.z = max(move_cmd.angular.z, -1.5)

        last_rotation = rotation
        cmd_vel.publish(move_cmd)
        rate.sleep()

    (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)

    while abs(rotation - goal_z) > 0.05:
        (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)
        if goal_z >= 0:
            if rotation <= goal_z and rotation >= goal_z - pi:
                move_cmd.linear.x = 0.00
                move_cmd.angular.z = 0.5
            else:
                move_cmd.linear.x = 0.00
                move_cmd.angular.z = -0.5
        else:
            if rotation <= goal_z + pi and rotation > goal_z:
                move_cmd.linear.x = 0.00
                move_cmd.angular.z = -0.5
            else:
                move_cmd.linear.x = 0.00
                move_cmd.angular.z = 0.5
        cmd_vel.publish(move_cmd)
        rate.sleep()


    rospy.loginfo("Stopping the robot...")
    cmd_vel.publish(Twist())
    is_ready.publish(Bool(True))
    rospy.sleep(2)


def shutdown():
    global cmd_vel
    cmd_vel.publish(Twist())
    rospy.sleep(1)


if __name__ == '__main__':
    global cmd_vel, is_ready
    rospy.init_node('movement', anonymous=False)
    sub = rospy.Subscriber("/movement_queue", Point, Callback)

    cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    is_ready = rospy.Publisher("/is_ready", Bool, queue_size=5)

    rospy.on_shutdown(shutdown)
    rospy.spin()


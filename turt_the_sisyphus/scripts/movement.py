import rospy
from geometry_msgs.msg import Twist, Point
import tf
from math import sqrt, pow, pi, atan2, sin, cos
from tf.transformations import euler_from_quaternion
import numpy as np
from std_msgs.msg import Bool


def get_odom(tf_listener, odom_frame, base_frame):
    try:
        (trans, rot) = tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
        rotation = euler_from_quaternion(rot)

    except (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.loginfo("TF Exception")
        return

    return (Point(*trans), rotation[2])

def Callback(data):
    global goals 
    goals.append(data)


def shutdown():
    global cmd_vel
    cmd_vel.publish(Twist())
    rospy.sleep(1)


def inRange(x, y, tolerance):
    return ((x[1] - y[1])**2 + (x[0] - y[0])**2)**0.5 < tolerance


def dist_callback(data):
    if data.z <= -1: 
        return
    global tf_listener, odom_frame, base_frame, ball_positions

    (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)

    ballWorldAngle = data.z + rotation
    xOrd = position.x + data.x * cos(ballWorldAngle)
    
    if xOrd > 1.5:
        return 

    yOrd = position.y + data.x * sin(ballWorldAngle)

    # for i in range(len(ball_positions)):
    #     if inRange([xOrd, yOrd], ball_positions[i], 0.6):
    #         ball_positions[i][0] = ball_positions[i][0] + xOrd / 2
    #         ball_positions[i][1] = ball_positions[i][1] + yOrd / 2
    #         return
    print(xOrd, yOrd)
    # ball_positions.append([xOrd, yOrd])



if __name__ == '__main__':
    rospy.init_node('movement', anonymous=False)
    sub = rospy.Subscriber("/movement_queue", Point, Callback)
    dist_sub = rospy.Subscriber("/ball_dist", Point, dist_callback)

    cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    coord_pub = rospy.Publisher("/ball_coord", Point, queue_size=5)
    is_moving = rospy.Publisher("/is_moving", Bool, queue_size=5)
    grip_pub = rospy.Publisher("/grip_coord", Point, queue_size=5)

    rospy.on_shutdown(shutdown)

    move_cmd = Twist()
    rate = rospy.Rate(20)
    tf_listener = tf.TransformListener()
    odom_frame = 'odom' 
    base_frame = None

    while base_frame is None:
        try:
            tf_listener.waitForTransform(odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                tf_listener.waitForTransform(odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.sleep(2)


    goals = []
    ball_positions = []
    while not rospy.is_shutdown():
        if len(goals) == 0: 
            is_moving.publish(Bool(False))
            continue 
        goal = goals.pop(0)
        print("next goal", goal)
        is_moving.publish(Bool(True))
        (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)
        last_rotation = 0
        linear_speed = 1
        angular_speed = 1
        (goal_x, goal_y, goal_z) = (goal.x, goal.y, goal.z)
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
            path_angle = atan2(goal_y - y_start, goal_x - x_start)

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
            is_moving.publish(Bool(False))
            rate.sleep()


        rospy.loginfo("Stopping the robot...")
        cmd_vel.publish(Twist())


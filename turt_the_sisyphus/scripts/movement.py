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


def insertGoal(x, y, angle=0):
    global goals
    goal = Point()
    goal.x = x 
    goal.y = y
    goal.z = angle
    goals.append(goal)


def distFromBall(position, x, y, dist):
    global goals 
    angle = atan2((y - position.y), (x - position.x))
    x = x - dist*cos(angle)
    y = y - dist*sin(angle)

    for point in goals:
        if inRange([x, y], [point.x, point.y], 0.1):
            return

    insertGoal(x, y, dist)


def schizoBozo(ballPos, position):
    if ballPos[0] < -0.35:
        return

    ballPos[1] = max(-1.15, min(1.15, ballPos[1]))

    if (position.x + 0.2) > ballPos[0]:
        if ballPos[1] > position.y:
            insertGoal(ballPos[0], ballPos[1] - 0.2)
        else:
            insertGoal(ballPos[0], ballPos[1] + 0.2)

    insertGoal(ballPos[0] - 0.2, ballPos[1])
    insertGoal(ballPos[0], ballPos[1])
    insertGoal(1, ballPos[1])


def dist_callback(data):
    if data.z <= -1 or abs(data.z) > 0.1: 
        return

    global tf_listener, odom_frame, base_frame, ball_positions, goals

    (position, rotation) = get_odom(tf_listener, odom_frame, base_frame)

    ballWorldAngle = data.z + rotation
    xOrd = position.x + data.x * cos(ballWorldAngle)
    
    if xOrd > 1.17:
        print("ball at", xOrd, " out of half")
        return 

    yOrd = position.y + data.x * sin(ballWorldAngle)

    for i in range(len(ball_positions)):
        if inRange([xOrd, yOrd], ball_positions[i][0], 0.1):
            point = Point()
            point.x = ball_positions[i][0][0] 
            point.y = ball_positions[i][0][1]
            coord_pub.publish(point)
            if len(ball_positions[i]) > 30: 
                ball_positions[i] = ball_positions[i][0:1]
                schizoBozo(ball_positions[i][0], position)
                print(ball_positions[i][0])
            ball_positions[i].append([xOrd, yOrd])
            if len(ball_positions[i]) < 6:
                return 
            sum_x = sum_y = 0
            for j in ball_positions[i]:
                sum_x += j[0]
                sum_y += j[1] 

            ball_positions[i][0][0] = sum_x / len(ball_positions[i])
            ball_positions[i][0][1] = sum_y / len(ball_positions[i])
            return

    print(xOrd, yOrd)
    ball_positions.append([[xOrd, yOrd]])
    point = Point()
    point.x = xOrd 
    point.y = yOrd
    coord_pub.publish(point)



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


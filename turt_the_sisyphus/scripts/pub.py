import rospy 
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool

move_list = [Point(1.155, 0, 0), 
             Point(0, 0.968, 0), Point(1.155, 0.979, 1), 
             Point(0, -0.91, 0), Point(1.155, -0.91, 0)]

def reverse(data):
    if data.data:
        if len(move_list) % 2 == 0:
            cmd = Twist()
            cmd.linear.x = -0.2
            # cmd.angular.z = 0.8
            cmd_vel.publish(cmd)
            rospy.sleep(0.5)
        if move_list:
            pub.publish(move_list[0])
    else:
        move_list.pop(0)

def shutdown():
    global cmd_vel 
    cmd_vel.publish(Twist())

if __name__ == "__main__":
    global cmd_vel
    rospy.init_node("pub", anonymous=True)
    pub = rospy.Publisher("/movement_queue", Point, queue_size=5)
    cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
    is_ready = rospy.Subscriber("/is_ready", Bool, reverse)
    rospy.on_shutdown(shutdown)
    rospy.sleep(1)
    pub.publish(move_list[0])
    rospy.spin()


import rospy 
from geometry_msgs.msg import Point

move_list = [Point(1.155, 0, 0), 
             Point(0, 0.97, 0), Point(1.155, 0.97, 1), 
             Point(0, -0.91, 0), Point(1.155, -0.91, 0),
             Point(0, 0, 0)]


if __name__ == "__main__":
    global cmd_vel
    rospy.init_node("pub", anonymous=True)
    pub = rospy.Publisher("/movement_queue", Point, queue_size=5)
    rospy.sleep(1)
    for i in range(len(move_list)):
        pub.publish(move_list[i])
    rospy.spin()


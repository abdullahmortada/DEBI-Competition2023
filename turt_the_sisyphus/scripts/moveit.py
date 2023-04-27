import sys
import rospy
import moveit_commander 
import geometry_msgs.msg
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

from movement import get_odom, tf_listener, odom_frame, base_frame
# import copy
# import moveit_msgs.msg 
# from math import tau



def openGripper():
    grip_goal = gripper_group.get_current_joint_values()
    grip_goal[0] = 0.009
    gripper_group.go(grip_goal, wait=True)
    gripper_group.stop()


def closeGripper():
    grip_goal = gripper_group.get_current_joint_values()
    grip_goal[0] = -0.0006
    gripper_group.go(grip_goal, wait=True)
    gripper_group.stop()


def goToAngle(angle):
    joint_goal = move_group.get_current_joint_values()
    for i in range(len(angle)):
        joint_goal[i] = angle[i]

    move_group.go(joint_goal, wait=True)
    move_group.stop()


def goToPose(pose):
    # move_group.set_goal_tolerance(0.02)
    move_group.set_pose_target(pose)
    move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()


def goCartesian(poseList):
    move_group.set_goal_tolerance(0.02)
    (plan, fraction) = move_group.compute_cartesian_path(
            poseList, 0.01, 0.0
            )
    print(fraction)
    move_group.execute(plan, wait=True)
    # move_group.stop()


def ballCallback(data):
    if is_moving:
        return 

    position, _ = get_odom(tf_listener, odom_frame, base_frame) 

    if ((data.x - position.x) ** 2 + (data.y - position.y)**2)**0.5 > 0.3:
        return 

    openGripper()

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = data.x - position.x
    pose_goal.position.y = data.y - position.y
    pose_goal.position.z = 0.1

    closeGripper()
    goToAngle([0,0,0,0])
    rate.sleep()


def moveStateCallback(data):
    global is_moving
    is_moving = data.data


if __name__ == "__main__":
    global is_moving, has_ball
    is_moving = False
    has_ball = False
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit", anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    rate = rospy.Rate(0.5)

    move_group = moveit_commander.MoveGroupCommander("arm")
    gripper_group = moveit_commander.MoveGroupCommander("gripper")
    dist_sub = rospy.Subscriber("/ball_coord", Point, ballCallback)
    moving_sub = rospy.Subscriber("/is_moving", Bool, moveStateCallback)

    goToAngle([0,0,0,0])
    rospy.spin()

import sys
import copy
import rospy
import moveit_commander 
import geometry_msgs.msg
# import moveit_msgs.msg 
# from std_msgs.msg import String 
# from moveit_commander.conversions import pose_to_list 
# from math import tau


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("moveit", anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()

move_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")

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
    (plan, _) = move_group.compute_cartesian_path(
            poseList, 0.01, 0.0
            )
    move_group.execute(plan, wait=True)
    move_group.stop()


ballpos = [0.9, 0.014, 0.027]
robpos = [0.72, 0.01, 0]

# display_trajectory_publisher = rospy.Publisher(
#     "/move_group/display_planned_path",
#     moveit_msgs.msg.DisplayTrajectory,
#     queue_size=20,
# )
# rospy.sleep(1)
goToAngle([0,0,0,0])

# openGripper()
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = ballpos[0] - robpos[0]
pose_goal.position.y = ballpos[1] - robpos[1]
pose_goal.position.z = 0.15
goToPose(pose_goal)


wpose = move_group.get_current_pose().pose 
wpose.position.z -= 0.11
# wpose.position.x += 0.01
waypoints = [copy.deepcopy(wpose)]
goCartesian(waypoints)

openGripper()
goToAngle([0,0,0,0])

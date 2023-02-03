#!/usr/bin/python3

############################################################

# Made to check the coordinated of xArm gripper and marker #

############################################################




import rospy
import sys
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import Int16MultiArray
from ar_track_alvar_msgs.msg import AlvarMarkers

def callback(data):
    global marker_pose

    marker_pose = data.markers[0].pose.pose

class PrintPose(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('check_positions', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")

    def printPose(self):
        xarm7 = self.xarm7
        current_pose = xarm7.get_current_pose().pose
        print('\nxArm pose:\n', round(current_pose.position.x, 2), round(current_pose.position.y, 2), round(current_pose.position.z, 2))
        print('\nMarker pose:\n', round(marker_pose.position.x, 2), round(marker_pose.position.y, 2), round(marker_pose.position.z, 2))

        error = geometry_msgs.msg.Pose()
        error.position.x = round(abs(current_pose.position.x - marker_pose.position.z), 2)
        error.position.y = round(abs(current_pose.position.y - marker_pose.position.y), 2)
        error.position.z = round(abs(current_pose.position.z - marker_pose.position.x), 2)
        # print('\nError:\n', error.position)

        move = geometry_msgs.msg.Pose()
        move.position.x = round(current_pose.position.x + marker_pose.position.z, 2)
        move.position.y = round(current_pose.position.y + marker_pose.position.y, 2)
        move.position.z = round(current_pose.position.z - marker_pose.position.x, 2)
        # print('\nMove:\n', move.position)

def main():

    # rospy.Subscriber("sections", Int16MultiArray, Sections)
    rospy.Subscriber("/ar_tf_marker", AlvarMarkers, callback)
    pose = PrintPose()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        pose.printPose()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
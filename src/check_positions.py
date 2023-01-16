#!/usr/bin/python3
import rospy
import sys
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import Int16MultiArray

sections_in = Int16MultiArray()
sections_in.data = [0,0,0,0]

# def Sections(sections):
#     global sections_in
#     if sections_in != sections:
#         print(sections)
#         sections_in = sections

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
        print(current_pose)

def main():

    # rospy.Subscriber("sections", Int16MultiArray, Sections)

    pose = PrintPose()
    pose.printPose()


if __name__ == '__main__':
    main()
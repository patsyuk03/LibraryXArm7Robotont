
###################################

# Work space for testing new code #

###################################

import rospy, sys, moveit_commander, yaml, tf, geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Int16MultiArray
from math import *
import numpy as np
from library.srv import Main, MainResponse
from std_srvs.srv import Trigger, TriggerResponse


class PNPbook(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")

    def xArm7Up(self):
        joint_goal = self.xarm7.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -0.68
        joint_goal[2] = 0
        joint_goal[3] = 0.83
        joint_goal[4] = 0
        joint_goal[5] = 1.5
        joint_goal[6] = 1
        self.xarm7.go(joint_goal, wait=True)

    def xArm7ToStart(self):
        joint_goal = self.xarm7.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -0.68
        joint_goal[2] = 0
        joint_goal[3] = 0.83
        joint_goal[4] = 0
        joint_goal[5] = 1.5
        joint_goal[6] = 0
        self.xarm7.go(joint_goal, wait=True)

def mainProgramme(req):
    rospy.loginfo('Got request from robotont')
    move.xArm7Up()
    return TriggerResponse(success=True, message="SPD")

def mainProgrammeStart(req):
    global move
    rospy.loginfo('Got request:%s', req)
    rospy.loginfo('Starting main programme.')
    move = PNPbook()
    rate = rospy.Rate(5)
    move.xArm7ToStart()
    rospy.loginfo('Going to start position.')
    robotont_serv = rospy.Service('robotont', Trigger, mainProgramme)
    return MainResponse('Finished')


def main():
    rospy.init_node('main', anonymous=True)
    director_serv = rospy.Service('main', Main, mainProgrammeStart)
    rospy.loginfo('Waiting for request (test).')
    rospy.spin()  

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
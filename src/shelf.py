
################################

# Looks for the shelf position #

################################

import rospy
import sys
import moveit_commander
from ar_track_alvar_msgs.msg import AlvarMarkers

def ShelfPosition(data):
    global shelf_positions

    for marker in data.markers:
        if marker.id not in shelf_positions.keys:
            shelf_positions[marker.id] = marker.pose.pose

class FindShelf(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('find_shelf', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")

    def lookForShelf(self):
        global x
        xarm7 = self.xarm7
        joint_goal = xarm7.get_current_joint_values()
        joint_goal[0] = 0.36
        joint_goal[1] = -0.68
        joint_goal[2] = 0
        joint_goal[3] = 0.83
        joint_goal[4] = 0
        joint_goal[5] = 1.5
        joint_goal[6] = 0

        x += 0.1

        xarm7.go(joint_goal, wait=True)

    def Gripper(self, state):
        gripper = self.gripper
        gripper_values = gripper.get_current_joint_values()
        if state == "open":
            gripper_values[0] = 0
        elif state == "close":
            gripper_values[0] = 0.33
        gripper.go(gripper_values, wait=True)

    def xArm7ToStart(self):
        xarm7 = self.xarm7
        joint_goal = xarm7.get_current_joint_values()
        joint_goal[0] = 0.36
        joint_goal[1] = -0.68
        joint_goal[2] = 0
        joint_goal[3] = 0.83
        joint_goal[4] = 0
        joint_goal[5] = 1.5
        joint_goal[6] = 0

        xarm7.go(joint_goal, wait=True)

def main():
    global shelf_positions

    shelf_positions = dict()

    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ShelfPosition)

    move = FindShelf()
    move.xArm7ToStart()
    move.Gripper("open")

    while len(shelf_positions) < 4:
        move.lookForShelf()
    
    print('Found shelf')
    move.xArm7ToStart()
    

if __name__ == '__main__':
    main()
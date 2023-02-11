
###################################

# Work space for testing new code #

###################################

import rospy
import sys
import moveit_commander
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers

def callback(data):
    global marker_pose
    if len(data.markers)>0:
        marker_pose = data.markers[0].pose.pose

class Motion(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('test_xarm', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")

    def xArm7ToMarker(self):
        xarm7 = self.xarm7
        current_pose = xarm7.get_current_pose().pose
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal = current_pose

        pose_goal.position.x = marker_pose.position.x
        pose_goal.position.y = marker_pose.position.y
        pose_goal.position.z = 0.15

        xarm7.set_pose_target(pose_goal)

        print('\nxArm pose:\n', round(current_pose.position.x, 2), round(current_pose.position.y, 2), round(current_pose.position.z, 2))
        print('\nMarker pose:\n', round(marker_pose.position.z, 2), round(marker_pose.position.y, 2), round(marker_pose.position.x, 2))
        print('\nxArm new pose:\n', round(pose_goal.position.x, 2), round(pose_goal.position.y, 2), round(pose_goal.position.z, 2))

        plan_success, traj, planning_time, error_code = xarm7.plan()
        xarm7.clear_pose_targets()
        self.ExecutePlan(traj)

    def ExecutePlan(self, plan):    
        xarm7 = self.xarm7
        xarm7.execute(plan, wait=True)
        xarm7.clear_pose_targets()

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

    def printPose(self):
        xarm7 = self.xarm7
        current_pose = xarm7.get_current_pose().pose
        print('\nxArm pose:\n', round(current_pose.position.x, 2), round(current_pose.position.y, 2), round(current_pose.position.z, 2))

        pose_goal = current_pose

        pose_goal.position.x += marker_pose.position.z
        pose_goal.position.y += marker_pose.position.y
        pose_goal.position.z = 0.05

        print('\nMarker pose:\n', round(marker_pose.position.z, 2), round(marker_pose.position.y, 2), round(marker_pose.position.x, 2))
        print('\nxArm new pose:\n', round(pose_goal.position.x, 2), round(pose_goal.position.y, 2), round(pose_goal.position.z, 2))

def main():
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)\

    move = Motion()
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        print("\n1-go to marker",
              "\n2-to start",
              "\n3-print poses")
        inp = input('?:')
        if inp == "1": move.xArm7ToMarker()
        elif inp == "2": move.xArm7ToStart()
        elif inp == "3": move.printPose()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

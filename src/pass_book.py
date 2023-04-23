import rospy, sys, moveit_commander, geometry_msgs.msg, tf

class PassTheBook(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.L_xarm7 = moveit_commander.MoveGroupCommander("L_xarm7")
        self.L_gripper = moveit_commander.MoveGroupCommander("L_xarm_gripper")
        self.R_xarm7 = moveit_commander.MoveGroupCommander("R_xarm7")
        self.R_gripper = moveit_commander.MoveGroupCommander("R_xarm_gripper")

        rospy.loginfo('PASS BOOK: Initialization complete.')

    def kiss(self):
        L_current_pose = self.L_xarm7.get_current_pose().pose
        L_pose_goal = L_current_pose
        L_pose_goal.position.y = L_current_pose.position.y + 0.08
        waypoints = list()
        waypoints.append(L_pose_goal)
        (traj, fraction) = self.L_xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.L_xarm7.execute(traj, wait=True)
        self.L_xarm7.clear_pose_targets()

        L_gripper_values = self.L_gripper.get_current_joint_values()
        L_gripper_values[0] = 0.65
        self.L_gripper.go(L_gripper_values, wait=True)

        R_gripper_values = self.L_gripper.get_current_joint_values()
        R_gripper_values[0] = 0
        self.R_gripper.go(R_gripper_values, wait=True)

        L_current_pose = self.L_xarm7.get_current_pose().pose
        L_pose_goal = L_current_pose
        L_pose_goal.position.y = L_current_pose.position.y - 0.07
        waypoints = list()
        waypoints.append(L_pose_goal)
        (traj, fraction) = self.L_xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.L_xarm7.execute(traj, wait=True)
        self.L_xarm7.clear_pose_targets()

    def RxArm7(self):
        joint_goal = self.R_xarm7.get_current_joint_values()
        joint_goal[0] = -1.571
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = -1.571
        joint_goal[6] = 0
        self.R_xarm7.go(joint_goal, wait=True)

        current_pose = self.R_xarm7.get_current_pose().pose
        current_pose.position.z = current_pose.position.z + 0.3
        waypoints = list()
        waypoints.append(current_pose)
        (traj, fraction) = self.R_xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.R_xarm7.execute(traj, wait=True)
        self.R_xarm7.clear_pose_targets()

    def LxArm7(self):
        joint_goal = self.L_xarm7.get_current_joint_values()
        joint_goal[0] = 1.571
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = -1.571
        joint_goal[6] = 0
        self.L_xarm7.go(joint_goal, wait=True)

        L_pose_goal = self.L_xarm7.get_current_pose().pose
        R_current_pose = self.R_xarm7.get_current_pose().pose
        L_pose_goal.position.x = R_current_pose.position.x
        L_pose_goal.position.y = R_current_pose.position.y - 0.5
        L_pose_goal.position.z = R_current_pose.position.z - 0.09
        waypoints = list()
        waypoints.append(L_pose_goal)
        (traj, fraction) = self.L_xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.L_xarm7.execute(traj, wait=True)
        self.L_xarm7.clear_pose_targets()


    def xArm7ToStart(self):
        joint_goal = self.L_xarm7.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -0.68
        joint_goal[2] = 0
        joint_goal[3] = 0.83
        joint_goal[4] = 0
        joint_goal[5] = 1.5
        joint_goal[6] = 0
        self.L_xarm7.go(joint_goal, wait=True)
        self.R_xarm7.go(joint_goal, wait=True)

def main():
    rospy.init_node('pass_book', anonymous=True)

    move = PassTheBook()
    move.xArm7ToStart()
    move.RxArm7()
    move.LxArm7()
    move.kiss()
    move.xArm7ToStart()


    # rospy.spin()  

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
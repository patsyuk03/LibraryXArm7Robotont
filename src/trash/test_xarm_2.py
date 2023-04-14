import rospy, sys, moveit_commander


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test_node', anonymous=True)
    robot = moveit_commander.RobotCommander()
    xarm7 = moveit_commander.MoveGroupCommander("xarm7")
    gripper = moveit_commander.MoveGroupCommander("xarm_gripper")

    # joint_goal = xarm7.get_current_joint_values()
    # joint_goal[0] = 0
    # joint_goal[1] = -0.7
    # joint_goal[2] = 0
    # joint_goal[3] = 0.8
    # joint_goal[4] = 0
    # joint_goal[5] = 1.5
    # joint_goal[6] = 0
    # xarm7.go(joint_goal, wait=True)
    pose_goal = xarm7.get_current_pose().pose
    pose_goal.position.x = 0.2
    pose_goal.position.y = 0.2
    pose_goal.position.z = 0.2

    xarm7.set_pose_target(pose_goal)
    plan_success, traj, planning_time, error_code = xarm7.plan()
    xarm7.execute(traj, wait=True)
    xarm7.clear_pose_targets()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit(0)
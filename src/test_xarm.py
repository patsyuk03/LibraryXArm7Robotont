import rospy
import sys
import moveit_commander

class StraightLineMotion(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('test_xarm', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")

    def lineMotion_1(self, dir):
        xarm7 = self.xarm7
        current_pose = xarm7.get_current_pose().pose
        waypoints = []
        if dir == 1:
            current_pose.position.z += 0.2 
        else:
            current_pose.position.z -= 0.2 
        waypoints.append(current_pose)
        (traj, fraction) = xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)
        xarm7.clear_pose_targets()
        return self.ExecutePlan(traj)

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

def main():
    move = StraightLineMotion()

    while True:
        print("Choose the type of line motion: \n",
        "1 -> cartesian path\n",
        "2 -> to start\n",
        "stop -> stop")
        inp = input()
        if inp == "1":
            print("Up or down: \n",
            "0 -> down\n",
            "1 -> up\n")
            dir = int(input())
            move.lineMotion_1(dir)
        elif inp == "2": move.xArm7ToStart()
        elif inp == "stop": break

if __name__ == '__main__':
    main()

import rospy
import sys
import moveit_commander

class StraightLineMotion(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('straight_line_motion', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")

    def lineMotion(self, direction):
        xarm7 = self.xarm7
        current_pose = xarm7.get_current_pose().pose

        waypoints = []
        if direction == "down": current_pose.position.z -= 0.2 
        elif direction == "up": current_pose.position.z += 0.2
        waypoints.append(current_pose)
        (traj, fraction) = xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)

        xarm7.clear_pose_targets()

        return traj

    def Gripper(self, state):
        gripper = self.gripper
        gripper_values = gripper.get_current_joint_values()
        if state == "open":
            gripper_values[0] = 0
        elif state == "close":
            gripper_values[0] = 0.25
        gripper.go(gripper_values, wait=True)

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
        print ("============ Press `Enter` to start Xarm7 movement")
        inp = input()
        if inp == "stop": break
        else: inp = "plan"

        # move.xArm7ToStart()
        move.Gripper("open")
        print ("============ Press `Enter` to close")
        inp = input()
        if inp == "stop": break
        else: inp = "plan"
        move.Gripper("close")

        # while inp == "plan":
        #     plan, bool_debug = move.xArm7ToObject() 

        #     print ("============ Press `ENTER` to execute a saved path, plan to replan")
        #     inp = input()
            
        # if inp == "stop": break
        # elif inp == "restart": main()
        
        # move.ExecutePlan(plan)
        # move.ExecutePlan(move.lineMotion("down"))
        # move.Gripper("close")


if __name__ == '__main__':
    main()
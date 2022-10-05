import rospy
import sys
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class PNPbook(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pnp_book', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")
    
    def xArm7ToObject(self):
        xarm7 = self.xarm7
        joint_goal = xarm7.get_current_joint_values()
        joint_goal[0] = 0.36
        joint_goal[1] = 0.42
        joint_goal[2] = 0
        joint_goal[3] = 1.5
        joint_goal[4] = 0
        joint_goal[5] = 1.11
        joint_goal[6] = 0

        xarm7.go(joint_goal, wait=True)

    def xArm7ToStorage(self):
        xarm7 = self.xarm7
        joint_goal = xarm7.get_current_joint_values()
        joint_goal[0] = 0.36
        joint_goal[1] = 0
        joint_goal[2] = 0.47
        joint_goal[3] = 0.93
        joint_goal[4] = 0
        joint_goal[5] = 0.94
        joint_goal[6] = 0

        xarm7.go(joint_goal, wait=True)

    def Gripper(self, state):
        gripper = self.gripper
        gripper_values = gripper.get_current_joint_values()
        if state == "open":
            gripper_values[0] = 0
        elif state == "close":
            gripper_values[0] = 0.74
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
    move = PNPbook()
    move.xArm7ToStart()
    move.Gripper("open")
    move.xArm7ToObject()
    move.Gripper("close")
    move.xArm7ToStorage()
    move.Gripper("open")
    move.xArm7ToStart()



    # while True:
    #     print ("============ Press `Enter` to start Xarm7 movement")
    #     inp = input()
    #     if inp == "stop": break
    #     else: inp = "plan"

    #     while inp == "plan":
    #         plan, bool_debug = move.xArm7ToObject() 

    #         print ("============ Press `ENTER` to execute a saved path, plan to replan")
    #         inp = input()
            
    #     if inp == "stop": break
    #     elif inp == "restart": main()
        
    #     move.ExecutePlan(plan)
    #     move.Gripper("close")
    #     inp = "plan"

    #     while inp == "plan":
    #         plan, bool_debug = move.xArm7ToStorage()

    #         print ("============ Press `ENTER` to execute a saved path, plan to replan")
    #         inp = input()
        
    #     if inp == "stop": break
    #     elif inp == "restart": main()

    #     move.ExecutePlan(plan)
    #     move.Gripper("open")
    #     move.xArm7ToStart()

if __name__ == '__main__':
    main()
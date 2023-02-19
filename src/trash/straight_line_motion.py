
############################################

# Check the xArm PythonSDK service request # 

############################################

import rospy
import sys
import moveit_commander
import time
from xarm.wrapper import XArmAPI

class StraightLineMotion(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('straight_line_motion', anonymous=True)

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
    
    def lineMotion_2(self, dir):
        arm = XArmAPI('192.168.1.196', is_radian=True)
        arm.motion_enable(enable=True)
        print('state:', arm.state)
        print('mode:', arm.mode)
        arm.set_mode(0)
        arm.set_state(state=0)

        current_pose = arm.get_position()[1]
        print("Current pose:", current_pose)
        if dir == 1:
            arm.set_position(
                x=current_pose[0], 
                y=current_pose[1], 
                z=current_pose[2]+200,
                roll=current_pose[3], 
                pitch=current_pose[4], 
                yaw=current_pose[5],
                speed=100, is_radian=True, wait=True
            )
        else:
            arm.set_position(
                x=current_pose[0], 
                y=current_pose[1], 
                z=current_pose[2]-200,
                roll=current_pose[3], 
                pitch=current_pose[4], 
                yaw=current_pose[5],
                speed=100, is_radian=True, wait=True
            )

        arm.disconnect()

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
        "2 -> service/client\n",
        "3 -> to start\n",
        "stop -> stop")
        inp = input()
        if inp == "1" or inp == "2":
            print("Up or down: \n",
            "0 -> down\n",
            "1 -> up\n")
            dir = int(input())
        if inp == "1":
            start = time.time()
            move.lineMotion_1(dir)
            end = time.time()
            print("Execution time:", end-start)
        elif inp == "2": 
            start = time.time()
            move.lineMotion_2(dir)
            end = time.time()
            print("Execution time:", end-start)
        elif inp == "3": move.xArm7ToStart()
        elif inp == "stop": break

if __name__ == '__main__':
    main()

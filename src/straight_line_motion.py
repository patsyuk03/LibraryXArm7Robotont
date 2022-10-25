import rospy
import sys
import moveit_commander
# import actionlib
# import geometry_msgs.msg
# import tf
# from xarm_msgs.srv import *
from xarm.wrapper import XArmAPI

class StraightLineMotion(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('straight_line_motion', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")

    def lineMotion_1(self):
        xarm7 = self.xarm7
        current_pose = xarm7.get_current_pose().pose
        waypoints = []
        current_pose.position.z -= 0.2 
        waypoints.append(current_pose)
        (traj, fraction) = xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)
        xarm7.clear_pose_targets()
        return self.ExecutePlan(traj)
    
    def lineMotion_2(self):
        arm = XArmAPI('192.168.1.196', is_radian=True)
        arm.motion_enable(enable=True)
        arm.set_mode(0)
        arm.set_state(state=0)
        arm.reset(wait=True)
        current_pose = arm.get_position()
        print("Current pose:",current_pose, '\n', current_pose[1])


        # arm.set_position(x=300, y=0, z=150, roll=-180, pitch=0, yaw=0, speed=100, is_radian=False, wait=True)
        # print(arm.get_position(), arm.get_position(is_radian=False))

        arm.reset(wait=True)
        arm.motion_enable(enable=False)
        arm.disconnect()

    # def lineMotion_2(self):
    #     xarm7 = self.xarm7
    #     current_pose = xarm7.get_current_pose().pose

    #     X = current_pose.position.x*1000
    #     Y = current_pose.position.y*1000
    #     Z = (current_pose.position.z - 0.2)*1000

    #     current_orientation = (
    #         current_pose.orientation.x,
    #         current_pose.orientation.y,
    #         current_pose.orientation.z,
    #         current_pose.orientation.w)

    #     euler = tf.transformations.euler_from_quaternion(current_orientation)

    #     roll = 3.14 + euler[0]
    #     pitch = euler[1]
    #     yaw = euler[2]

    #     client = actionlib.SimpleActionClient('/xarm/move_servo_cart', )
    #     client.wait_for_server()
    #     goal = [X, Y, Z, roll, pitch, yaw]
    #     client.send_goal(goal)
    #     client.wait_for_result(rospy.Duration.from_sec(5.0))

    # def lineMotion_2(self):
    #     rospy.wait_for_service('/xarm/move_line')
    #     rospy.loginfo("Waiting for service /xarm/move_line")
    #     rospy.set_param('/xarm/wait_for_finish', True)
    #     rospy.loginfo("Connected /xarm/move_line")
    #     motion_en = rospy.ServiceProxy('/xarm/motion_ctrl', SetAxis)
    #     set_mode = rospy.ServiceProxy('/xarm/set_mode', SetInt16)
    #     set_state = rospy.ServiceProxy('/xarm/set_state', SetInt16)
    #     get_position = rospy.ServiceProxy('/xarm/get_position_rpy', GetFloat32List)
    #     rospy.loginfo("Connected: \n/xarm/motion_ctrl\n/xarm/set_mode\n/xarm/set_state\n/xarm/get_position_rpy")
    #     set_mode(0)
    #     set_state(0)
    #     rospy.loginfo("Set mode and state")
    #     start_pose = list(get_position().datas)
    #     print("Satrt pose:", start_pose)
    #     move_line = rospy.ServiceProxy('/xarm/move_servo_cart', Move)
    #     rospy.loginfo("/xarm/move_servo_cart: Move")
    #     req = MoveRequest() 
    #     rospy.loginfo("Move request")
    #     req.pose = start_pose
    #     req.pose[2] -= 0.2
    #     # req.mvvelo = 0
    #     # req.mvacc = 0
    #     # req.mvtime = 0 
    #     move_line(req)

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
        "2 -> servis/client\n",
        "3 -> to start\n",
        "stop -> stop")
        inp = input()
        if inp == "1": move.lineMotion_1()
        elif inp == "2": move.lineMotion_2()
        elif inp == "3": move.xArm7ToStart()
        elif inp == "stop": break

if __name__ == '__main__':
    main()

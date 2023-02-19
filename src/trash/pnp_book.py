
##########################################

# Position of box and shelf is hardcoded #

##########################################


import rospy
import sys
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Int16MultiArray

sections_in = [0,0,0,0]

storage_pose_1 = geometry_msgs.msg.Pose()
storage_pose_1.orientation.x = -0.9998241931924544
storage_pose_1.orientation.y = -0.014623415973421643
storage_pose_1.orientation.z = -0.007084661776556929
storage_pose_1.orientation.w = 0.009356600869873697
storage_pose_1.position.x = 0.19633979114494593
storage_pose_1.position.y = 0.051652752356458345
storage_pose_1.position.z = 0.3726752687552739

storage_pose_2 = geometry_msgs.msg.Pose()
storage_pose_2.orientation.x = 0.7194131851654499
storage_pose_2.orientation.y = -0.1275038267527848
storage_pose_2.orientation.z = 0.6632235297299842
storage_pose_2.orientation.w = 0.16224053989678128
storage_pose_2.position.x = 0.5351547843227856
storage_pose_2.position.y = -0.23586754598015158
storage_pose_2.position.z = 0.40315139868781114

object_pose = geometry_msgs.msg.Pose()
object_pose.orientation.x = -0.9992377390999495
object_pose.orientation.y = -0.017418570891527443
object_pose.orientation.z = -0.03378727441333642
object_pose.orientation.w = 0.008885619519008434
object_pose.position.x = 0.4906888143562935
object_pose.position.y = 0.13525305396465243
object_pose.position.z = 0.29711573230546273

def Sections(data):
    global sections_in, sections
    if sections_in != data.data:
        sections = data.data
        print(sections)
        sections_in = sections

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
    
    def xArm7ToObject(self, positions, section):
        xarm7 = self.xarm7
        pose_goal = geometry_msgs.msg.Pose()
        current_pose = xarm7.get_current_pose().pose
        pose_goal = current_pose

        pose_goal.orientation.x = positions.orientation.x
        pose_goal.orientation.y = positions.orientation.y
        pose_goal.orientation.z = positions.orientation.z
        pose_goal.orientation.w = positions.orientation.w

        pose_goal.position.x = positions.position.x
        pose_goal.position.y = positions.position.y
        pose_goal.position.z = positions.position.z

        xarm7.set_pose_target(pose_goal)

        plan_success, traj, planning_time, error_code = xarm7.plan()
        xarm7.clear_pose_targets()
        self.ExecutePlan(traj)

        joint_goal = xarm7.get_current_joint_values()
        joint_goal[0] -= 0.1745*section
        joint_goal[6] -= 0.1745*section
        xarm7.go(joint_goal, wait=True)

    def xArm7ToStorage(self, positions):
        xarm7 = self.xarm7
        pose_goal = geometry_msgs.msg.Pose()
        current_pose = xarm7.get_current_pose().pose
        pose_goal = current_pose

        pose_goal.orientation.x = positions.orientation.x
        pose_goal.orientation.y = positions.orientation.y
        pose_goal.orientation.z = positions.orientation.z
        pose_goal.orientation.w = positions.orientation.w

        pose_goal.position.x = positions.position.x
        pose_goal.position.y = positions.position.y
        pose_goal.position.z = positions.position.z

        xarm7.set_pose_target(pose_goal)

        plan_success, traj, planning_time, error_code = xarm7.plan()
        xarm7.clear_pose_targets()

        self.ExecutePlan(traj)

    def lineMotion(self, direction):
        xarm7 = self.xarm7
        current_pose = xarm7.get_current_pose().pose

        waypoints = []
        if direction == "down": 
            current_pose.position.z -= 0.2 
        elif direction == "up": 
            current_pose.position.z += 0.35
        elif direction == "forward": 
            current_pose.position.x += 0.1
            current_pose.position.z -= 0.05
        elif direction == "backward": 
            current_pose.position.x -= 0.1
            current_pose.position.z += 0.05
        waypoints.append(current_pose)
        (traj, fraction) = xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)

        xarm7.clear_pose_targets()

        self.ExecutePlan(traj)

    def Gripper(self, state):
        gripper = self.gripper
        gripper_values = gripper.get_current_joint_values()
        if state == "open":
            gripper_values[0] = 0
        elif state == "close":
            gripper_values[0] = 0.33
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
    global storage_pose_1, storage_pose_2, object_pose
    rospy.Subscriber("sections", Int16MultiArray, Sections)

    move = PNPbook()
    for section in range(len(sections)):
        if sections[section] == 1:
            # inp = input("Press `Enter` to start Xarm7 movement or `stop` to stop: ")
            # if inp == "stop": break

            move.xArm7ToStart()
            move.Gripper("open")

            move.xArm7ToObject(object_pose, section)
            move.lineMotion("down")
            move.Gripper("close")
            move.lineMotion("up")

            move.xArm7ToStorage(storage_pose_1)
            move.xArm7ToStorage(storage_pose_2)
            move.lineMotion("forward")
            move.Gripper("open")
            move.lineMotion("backward")

            move.xArm7ToStart()

if __name__ == '__main__':
    main()
######################################

# Box with books detected by markers #

######################################



import rospy
import sys
import moveit_commander
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers

def callback(data):
    global book, detected_id
    for marker in data.markers:
        if marker.id == 14 and marker.id != detected_id:
            book = marker.pose.pose
            print(book)
            detected_id = marker.id

class PNPbook(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pnp_book_v2', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")
    
    def xArm7ToObject(self, positions):
        xarm7 = self.xarm7
        pose_goal = geometry_msgs.msg.Pose()
        current_pose = xarm7.get_current_pose().pose
        pose_goal = current_pose
        print(pose_goal)

        # pose_goal.orientation.x = positions.orientation.x
        # pose_goal.orientation.y = positions.orientation.y
        # pose_goal.orientation.z = positions.orientation.z
        # pose_goal.orientation.w = positions.orientation.w

        # pose_goal.position.x = positions.position.x
        # pose_goal.position.y = positions.position.y
        # pose_goal.position.z = positions.position.z+0.1

        # xarm7.set_pose_target(pose_goal)

        # plan_success, traj, planning_time, error_code = xarm7.plan()
        # xarm7.clear_pose_targets()
        # self.ExecutePlan(traj)

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
            current_pose.position.z -= 0.1 
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

    def xArm7ToBox(self):
        xarm7 = self.xarm7
        joint_goal = xarm7.get_current_joint_values()
        joint_goal[0] = 0.0175
        joint_goal[1] = -0.24
        joint_goal[2] = -0.15
        joint_goal[3] = 1.43
        joint_goal[4] = 0
        joint_goal[5] = 1.5
        joint_goal[6] = 0

        xarm7.go(joint_goal, wait=True)


def main():
    global book, detected_id
    book, detected_id = False, 0

    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)

    move = PNPbook()
    move.xArm7ToStart()
    move.Gripper("open")
    # move.xArm7ToBox()
    
    while not rospy.is_shutdown():
        if book != False:
            print('going for a book')
            # inp = input("Press `Enter` to start Xarm7 movement or `stop` to stop: ")
            # if inp == "stop": break

            # move.xArm7ToStart()
            # move.Gripper("open")
            # move.xArm7ToBox()

            move.xArm7ToObject(book)
            # move.lineMotion("down")
            # move.Gripper("close")
            # move.lineMotion("up")

            # move.xArm7ToStorage(storage_pose_1)
            # move.lineMotion("forward")
            # move.Gripper("open")
            # move.lineMotion("backward")

            # move.xArm7ToStart()
            book = False
        

if __name__ == '__main__':
    main()
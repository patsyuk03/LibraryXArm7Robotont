######################################

# Box with books detected by markers #

######################################



import rospy
import sys
import moveit_commander
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Int16MultiArray

def BookPosition(data):
    global book_positions, sections

    for marker in data.markers:
        # print(book_positions.keys(), type(book_positions.keys()))
        if marker.id not in list(book_positions.keys()) and marker.id in sections:
            book_positions[marker.id] = marker.pose.pose

def Sections(data):
    global sections_in, sections

    if sections_in != data.data:
        sections = data.data
        sections_in = sections

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
        current_pose = xarm7.get_current_pose().pose
        pose_goal = current_pose

        pose_goal.position.x = positions.position.x
        pose_goal.position.y = positions.position.y
        pose_goal.position.z = 0.15

        xarm7.set_pose_target(pose_goal)

        plan_success, traj, planning_time, error_code = xarm7.plan()
        xarm7.clear_pose_targets()
        self.ExecutePlan(traj)

    def xArm7ToStorage(self, positions):
        xarm7 = self.xarm7
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
        joint_goal[0] = 0
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
    global book_positions, sections, sections_in

    book_positions = dict()
    sections = list()
    sections_in = list()

    rospy.Subscriber("/arm/ar_tf_marker", AlvarMarkers, BookPosition)
    rospy.Subscriber("sections", Int16MultiArray, Sections)

    move = PNPbook()
    move.xArm7ToStart()
    move.Gripper("open")

    rate = rospy.Rate(5)
    
    while not rospy.is_shutdown():
        if sections:
            # move.xArm7ToBox()
            if set(book_positions.keys()) == set(sections):
                print('Found box')
                for id in sections:
                    book = book_positions[id]
                    print(f'Going for a book {id}')

                    move.xArm7ToObject(book)
                    move.lineMotion("down")
                    move.Gripper("close")
                    move.lineMotion("up")

                    # move.xArm7ToStorage()
                    # move.lineMotion("forward")
                    # move.Gripper("open")
                    # move.lineMotion("backward")

                    move.xArm7ToStart()
                    move.Gripper("open")
            else:
                print(set(book_positions.keys()))
                move.xArm7ToStart()
                move.Gripper("open")
                print('there is no box')

        rate.sleep()
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
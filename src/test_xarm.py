
###################################

# Work space for testing new code #

###################################

import rospy
import sys
import moveit_commander
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
import yaml, math

def getAngle(a, c):
    ang = math.atan2(c[1], c[0]) - math.atan2(a[1], a[0])
    return ang + 2*math.pi if ang < 0 else ang

class Motion(object):
    def __init__(self):
        rospy.init_node('test_xarm', anonymous=True)

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")

    def xArm7ToMarker(self, marker_pose):
        xarm7 = self.xarm7
        current_pose = xarm7.get_current_pose().pose
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal = current_pose

        angle = getAngle((marker_pose.position.x, 0), (marker_pose.position.x, marker_pose.position.y))
        print(angle, math.degrees(angle))

        pose_goal.position.x = marker_pose.position.x - 0.1*math.cos(angle)
        pose_goal.position.y = marker_pose.position.y - 0.1*math.sin(angle)
        pose_goal.position.z = marker_pose.position.z + 0.1

        xarm7.set_pose_target(pose_goal)

        plan_success, traj, planning_time, error_code = xarm7.plan()
        xarm7.clear_pose_targets()
        self.ExecutePlan(traj)

    def GetShelfPosition(self):
        global shelf_position
        with open('src/library/yaml/shelf_position.yaml') as file:
            data = yaml.load(file, Loader=yaml.FullLoader)
            for i in range(len(data)):
                    id = list(data[i].keys())[0]
                    shelf_position[id] = geometry_msgs.msg.Pose()
                    shelf_position[id].position.x = data[i][id]['position']['x']
                    shelf_position[id].position.y = data[i][id]['position']['y']
                    shelf_position[id].position.z = data[i][id]['position']['z']
                    shelf_position[id].orientation.x = data[i][id]['orientation']['x']
                    shelf_position[id].orientation.y = data[i][id]['orientation']['y']
                    shelf_position[id].orientation.z = data[i][id]['orientation']['z']
                    shelf_position[id].orientation.w = data[i][id]['orientation']['w']

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
    global shelf_position
    shelf_position = dict()
    # rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)

    move = Motion()
    rate = rospy.Rate(5)
    move.GetShelfPosition()

    while not rospy.is_shutdown():
        print("\n1-go to shelf",
              "\n2-to start",)
        inp = input('?:')
        if inp == "1": move.xArm7ToMarker(shelf_position[1])
        elif inp == "2": move.xArm7ToStart()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

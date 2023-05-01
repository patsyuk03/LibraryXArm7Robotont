
################################

# Looks for the shelf position #

################################

import rospy, sys, moveit_commander
from ar_track_alvar_msgs.msg import AlvarMarkers
import time, yaml, os
from std_srvs.srv import Trigger, TriggerResponse


shelf_positions = list()
found_marker_ids = list()
marker_ids = [1, 6, 7]
move_group = "xarm7"
marker_source = "arm_1/ar_tf_marker"

def ShelfPosition(data):
    global shelf_positions
    for marker in data.markers:
        if marker.id not in found_marker_ids and marker.id in marker_ids:
            found_marker_ids.append(marker.id)

            position_x = marker.pose.pose.position.x
            position_y = marker.pose.pose.position.y
            position_z = marker.pose.pose.position.z
            position = {'x':position_x, 'y':position_y, 'z':position_z}

            orientation_x = marker.pose.pose.orientation.x
            orientation_y = marker.pose.pose.orientation.y
            orientation_z = marker.pose.pose.orientation.z
            orientation_w = marker.pose.pose.orientation.w
            orientation = {'x':orientation_x, 'y':orientation_y, 'z':orientation_z, 'w':orientation_w}

            shelf_positions.append({marker.id:{'position':position, 'orientation':orientation}})

class FindShelfClass(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.xarm7 = moveit_commander.MoveGroupCommander(move_group)
        self.writeToYAML()
    
    def findShelf(self):
        joint_goal_0 = 0
        turn = 'CLOCKWISE'

        self.xArm7ToStart()
        rospy.loginfo("FIND SHELF: Mooving to start position.")
        rate = rospy.Rate(10)
        start_time = time.time()
        while not rospy.is_shutdown():
            if len(shelf_positions)<len(marker_ids):
                if time.time()-start_time < 120:
                    self.lookForShelf(joint_goal_0)
                    rospy.loginfo("FIND SHELF: Looking for the shelf.")
                    if joint_goal_0 == 0:
                        joint_goal_0 = -3 if turn == 'CLOCKWISE' else 3
                    else:
                        turn = 'CLOCKWISE' if joint_goal_0 > 0 else 'COUNTERCLOCKWISE'
                        joint_goal_0 = 0
                else:
                    outcome = "FIND SHELF: No shelf was found."
                    break
            else:
                outcome = "FIND SHELF: Found shelf."
                self.writeToYAML()
                break
            rate.sleep()
        rospy.loginfo(outcome)
        self.xArm7ToStart()

    def lookForShelf(self, joint_goal_0):
        joint_goal = self.xarm7.get_current_joint_values()
        joint_goal[0] = joint_goal_0
        joint_goal[1] = -0.68
        joint_goal[2] = 0
        joint_goal[3] = 1.29
        joint_goal[4] = 0
        joint_goal[5] = 1.5
        joint_goal[6] = 0
        self.xarm7.go(joint_goal, wait=True)

    def xArm7ToStart(self):
        joint_goal = self.xarm7.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -0.68
        joint_goal[2] = 0
        joint_goal[3] = 0.83
        joint_goal[4] = 0
        joint_goal[5] = 1.5
        joint_goal[6] = 0
        self.xarm7.go(joint_goal, wait=True)
    
    def writeToYAML(self):
        global shelf_positions
        dir = os.path.dirname(__file__)
        rel_path = "./../yaml/shelf_position.yaml"
        abs_file_path = os.path.join(dir, rel_path)
        with open(abs_file_path, 'w') as file:
            data = yaml.dump(shelf_positions, file)

def findShelfStart(req):
    rospy.loginfo('FIND SHELF: Got request.')
    find_shelf = FindShelfClass()
    find_shelf.findShelf()
    return TriggerResponse(success=True)

def main():
    global move_group
    rospy.init_node('find_shelf', anonymous=True)
    if rospy.get_param('dual_arm'):
        move_group = "L_xarm7"
    rospy.Subscriber(marker_source, AlvarMarkers, ShelfPosition)
    s = rospy.Service('find_shelf', Trigger, findShelfStart)
    rospy.loginfo('FIND SHELF: Waiting for request (find shelf).')
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
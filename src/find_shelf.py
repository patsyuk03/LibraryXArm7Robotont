
################################

# Looks for the shelf position #

################################

import rospy
import sys
import moveit_commander
from ar_track_alvar_msgs.msg import AlvarMarkers
import time, yaml
from std_msgs.msg import String

def DirectorCallback(data):
    global director
    director = data.data

def ShelfPosition(data):
    global shelf_positions, found_marker_ids, marker_ids

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

class FindShelf(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")

    def lookForShelf(self, joint_goal_0):
        xarm7 = self.xarm7
        joint_goal = xarm7.get_current_joint_values()
        joint_goal[0] = joint_goal_0
        joint_goal[1] = -0.68
        joint_goal[2] = 0
        joint_goal[3] = 1.29
        joint_goal[4] = 0
        joint_goal[5] = 1.5
        joint_goal[6] = 0

        xarm7.go(joint_goal, wait=True)

    def writeToYAML(self):
        global shelf_positions
        with open('src/library/yaml/shelf_position.yaml', 'w') as file:
            data = yaml.dump(shelf_positions, file)

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
    global shelf_positions, found_marker_ids, marker_ids, director

    shelf_positions = list()
    found_marker_ids = list()
    marker_ids = [1, 6, 7]
    joint_goal_0 = 3
    turn = 'right'
    director = 'LOOK FOR SHELF'

    rospy.init_node('find_shelf', anonymous=True)
    rospy.Subscriber("director", String, DirectorCallback)
    rospy.Subscriber("/arm/ar_tf_marker", AlvarMarkers, ShelfPosition)
    pub = rospy.Publisher('find_shelf', String, queue_size=1)

    while not rospy.is_shutdown():
        pub.publish("WAITING")
        if director == "LOOK FOR SHELF":
            move = FindShelf()
            rate = rospy.Rate(10)

            move.xArm7ToStart()
            rospy.loginfo("Mooving to start position.")

            start_time = time.time()
            while not rospy.is_shutdown():
                if len(shelf_positions)<len(marker_ids):
                    if time.time()-start_time < 120:
                        move.lookForShelf(joint_goal_0)
                        rospy.loginfo("Mooving to 'look for book' position.")
                        if turn == 'left':
                            joint_goal_0 += 0.1
                            if joint_goal_0 > 3:
                                turn = 'right'
                        else:
                            joint_goal_0 -= 0.1
                            if joint_goal_0 < -3:
                                turn = 'left'
                    else:
                        outcome = "No shelf was found."
                        break
                else:
                    outcome = "Found shelf."
                    move.writeToYAML()
                    break
                rate.sleep()
            rospy.loginfo(outcome)
            # while director == "LOOK FOR SHELF":
            #     pub.publish(outcome)
            move.xArm7ToStart()
            break

    

if __name__ == '__main__':
    main()
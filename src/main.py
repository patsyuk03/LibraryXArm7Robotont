######################################

# Box with books detected by markers #

######################################



import rospy, sys, moveit_commander, yaml, tf, geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String
from math import *
import numpy as np

def DirectorCallback(data):
    global director
    director = data.data

def BookPositionCallback(data):
    global book_positions, sections

    for marker in data.markers:
        # print(book_positions.keys(), type(book_positions.keys()))
        if marker.id not in list(book_positions.keys()) and marker.id in sections:
            book_positions[marker.id] = marker.pose.pose

def SectionsCallback(data):
    global sections_in, sections

    if sections_in != data.data:
        sections = data.data
        sections_in = sections

def GetShelfPosition():
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

def getCoordinates(id):
    global shelf_position

    markers_y = list()
    markers_x = list()
    for marker in list(shelf_position.keys()):
        markers_y.append(shelf_position[marker].position.y)
        markers_x.append(shelf_position[marker].position.x)

    # finding equation of the line on which shelf is located
    coeff = np.polyfit(markers_x, markers_y, 1)
    print("Coeffs:", coeff)
    # finding the equation of the line on whish xArm7 will stop before putting the book
    dist = 0.2/sin(pi/2 - abs(atan(coeff[0])))
    print("Dist, angle:", dist, atan(coeff[0]))
    b = coeff[1]-dist if shelf_position[id].position.y>0 else coeff[1]+dist
    print("b:", b)
    dist_2 = sqrt(abs(shelf_position[id].position.x**2+(shelf_position[id].position.y-b)**2-0.04))
    a = coeff[0]**2+1
    c = -dist_2**2
    x = [-sqrt(-4*a*c)/(2*a), sqrt(-4*a*c)/(2*a)]
    y = [coeff[0]*x[0]+b, coeff[0]*x[1]+b]
    for i in range(2):
        r = round(sqrt((shelf_position[id].position.x-x[i])**2 + (shelf_position[id].position.y-y[i])**2), 1)
        print(r)
        if r == 0.2:
            print(x[i], y[i])
            angle1 = atan(coeff[0])
            angle2 = atan2(b, 0) - atan2(y[i], x[i])
            angle3 = angle1 + angle2 #if coeff[0]<0 else abs(angle1) - abs(angle2)
            print(angle1, angle2, angle3)
            return [x[i], y[i], angle3]

class PNPbook(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.xarm7 = moveit_commander.MoveGroupCommander("xarm7")
        self.gripper = moveit_commander.MoveGroupCommander("xarm_gripper")
    
    def xArm7ToObject(self, id):
        global book_positions

        gripper_values = self.gripper.get_current_joint_values()
        gripper_values[0] = 0
        self.gripper.go(gripper_values, wait=True)

        current_pose = self.xarm7.get_current_pose().pose
        pose_goal = current_pose
        pose_goal.position.x = book_positions[id].position.x + 0.1
        pose_goal.position.y = book_positions[id].position.y 
        pose_goal.position.z = book_positions[id].position.z + 0.2
        self.xarm7.set_pose_target(pose_goal)
        plan_success, traj, planning_time, error_code = self.xarm7.plan()
        self.xarm7.execute(traj, wait=True)
        self.xarm7.clear_pose_targets()

        current_pose = self.xarm7.get_current_pose().pose
        current_pose.position.z -= 0.11
        waypoints = list()
        waypoints.append(current_pose)
        (traj, fraction) = self.xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.xarm7.execute(traj, wait=True)
        self.xarm7.clear_pose_targets()

        gripper_values = self.gripper.get_current_joint_values()
        gripper_values[0] = 0.45
        self.gripper.go(gripper_values, wait=True)

        current_pose = self.xarm7.get_current_pose().pose
        current_pose.position.z += 0.1
        waypoints = list()
        waypoints.append(current_pose)
        (traj, fraction) = self.xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.xarm7.execute(traj, wait=True)
        self.xarm7.clear_pose_targets()

    def xArm7ToShelf(self, id):
        global shelf_position

        joint_goal = self.xarm7.get_current_joint_values()
        joint_goal[0] = atan2(shelf_position[id].position.y, shelf_position[id].position.x)
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0
        joint_goal[4] = 0
        joint_goal[5] = -1.571
        joint_goal[6] = 0
        self.xarm7.go(joint_goal, wait=True)

        current_pose = self.xarm7.get_current_pose().pose
        current_pose.position.z = shelf_position[id].position.z + 0.1
        waypoints = list()
        waypoints.append(current_pose)
        (traj, fraction) = self.xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.xarm7.execute(traj, wait=True)
        self.xarm7.clear_pose_targets()

        coords = getCoordinates(id)
        pose_goal = current_pose
        pose_goal.position.x = coords[0]
        pose_goal.position.y = coords[1]

        quaternion1 = (
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w)
        quaternion2 = tf.transformations.quaternion_from_euler(coords[2], 0, 0)
        quaternion = tf.transformations.quaternion_multiply(quaternion1, quaternion2)

        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

        self.xarm7.set_pose_target(pose_goal)
        plan_success, traj, planning_time, error_code = self.xarm7.plan()
        self.xarm7.execute(traj, wait=True)
        self.xarm7.clear_pose_targets()

        current_pose = self.xarm7.get_current_pose().pose
        current_pose.position.x = shelf_position[id].position.x
        current_pose.position.y = shelf_position[id].position.y
        waypoints = list()
        waypoints.append(current_pose)
        (traj, fraction) = self.xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.xarm7.execute(traj, wait=True)
        self.xarm7.clear_pose_targets()

        gripper_values = self.gripper.get_current_joint_values()
        gripper_values[0] = 0
        self.gripper.go(gripper_values, wait=True)

        current_pose = self.xarm7.get_current_pose().pose
        current_pose.position.x = coords[0]
        current_pose.position.y = coords[1]
        waypoints = list()
        waypoints.append(current_pose)
        (traj, fraction) = self.xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.xarm7.execute(traj, wait=True)
        self.xarm7.clear_pose_targets()

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


def main():
    global book_positions, sections, sections_in, director, shelf_position

    book_positions = dict()
    shelf_position = dict()
    sections = list()
    sections_in = list()
    director = 'MAIN PROGRAM'#"NONE"

    rospy.init_node('main', anonymous=True)

    rospy.Subscriber("director", String, DirectorCallback)
    rospy.Subscriber("/arm/ar_tf_marker", AlvarMarkers, BookPositionCallback)
    rospy.Subscriber("sections", Int16MultiArray, SectionsCallback)
    pub = rospy.Publisher('main', String, queue_size=1)
    

    # while not rospy.is_shutdown():
    pub.publish("WAITING")
    if director == 'MAIN PROGRAM':

        GetShelfPosition()
        move = PNPbook()
        rate = rospy.Rate(5)

        move.xArm7ToStart()
        rospy.loginfo('Going to start position')
        while not rospy.is_shutdown():
            rospy.loginfo_once('Waitiong for sections to be published')
            if sections:
                rospy.loginfo_once('Sections are published')
                if set(book_positions.keys()) == set(sections):
                    rospy.loginfo('Found box')
                    for id in sections:
                        rospy.loginfo(f'Going for a book {id}')

                        move.xArm7ToObject(id)
                        move.xArm7ToStart()

                        move.xArm7ToShelf(id)
                        move.xArm7ToStart()
                        if rospy.is_shutdown(): break

                    rospy.loginfo('Done.')
                else:
                    print(set(book_positions.keys()))
                    move.xArm7ToStart()
                    rospy.loginfo('There is no box.')
                    
                rate.sleep()
    rospy.spin()
            
        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
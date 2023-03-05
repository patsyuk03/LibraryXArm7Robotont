
###################################

# Work space for testing new code #

###################################

import rospy
import sys
import moveit_commander
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
import yaml, tf
import numpy as np
from math import *


def getAngle(c, a):
    ang = atan2(c[1], c[0]) - atan2(a[1], a[0])
    return ang + 2*pi if ang < 0 else ang

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

        # angle = getAngle(marker_pose.position.x, marker_pose.position.y)
        # print(angle)
        # pose_goal.position.x = 0.2*cos(angle)
        # pose_goal.position.y = 0.2*sin(angle)
        pose_goal.position.x = marker_pose.position.x
        pose_goal.position.y = marker_pose.position.y
        pose_goal.position.z = marker_pose.position.z + 0.1

        # marker_orientation = (
        # current_pose.orientation.x,
        # current_pose.orientation.y,
        # current_pose.orientation.z,
        # current_pose.orientation.w)

        # euler = tf.transformations.euler_from_quaternion(marker_orientation)

        # roll = euler[0]
        # pitch = euler[1]-1.571
        # yaw = euler[2]

        # quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

        # pose_goal.orientation.x = quaternion[0]
        # pose_goal.orientation.y = quaternion[1]
        # pose_goal.orientation.z = quaternion[2]
        # pose_goal.orientation.w = quaternion[3]

        xarm7.set_pose_target(pose_goal)

        plan_success, traj, planning_time, error_code = xarm7.plan()
        xarm7.clear_pose_targets()
        self.ExecutePlan(traj)

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
        self.xarm7.clear_pose_targets()
        self.ExecutePlan(traj)

        coords = self.getCoordinates(id)
        pose_goal = geometry_msgs.msg.Pose()
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
        self.xarm7.clear_pose_targets()
        self.ExecutePlan(traj)
       
        current_pose = self.xarm7.get_current_pose().pose
        current_pose.position.x = shelf_position[id].position.x
        current_pose.position.y = shelf_position[id].position.y
        waypoints = list()
        waypoints.append(current_pose)
        (traj, fraction) = self.xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.xarm7.clear_pose_targets()
        self.ExecutePlan(traj)


    def testRotation(self, id):
        xarm7 = self.xarm7
        current_pose = xarm7.get_current_pose().pose
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal = current_pose

        coords = self.getCoordinates(id)

        quaternion1 = (
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion1)

        quaternion2 = tf.transformations.quaternion_from_euler(coords[2], 0, 0)

        quaternion = tf.transformations.quaternion_multiply(quaternion1, quaternion2)

        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

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

    def getCoordinates(self, id):
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
            if r == 0.2:
                print(x[i], y[i])
                angle1 = atan(coeff[0])
                angle2 = atan2(b, 0) - atan2(y[i], x[i])
                angle3 = abs(angle2) - abs(angle1) if coeff[0]<0 else abs(angle1) - abs(angle2)
                print(angle1, angle2, angle3)
                return [x[i], y[i], angle3]

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


    def shelfMove(self, id):
        current_pose = self.xarm7.get_current_pose().pose
        current_pose.position.x = shelf_position[id].position.x
        current_pose.position.y = shelf_position[id].position.y
        waypoints = list()
        waypoints.append(current_pose)
        (traj, fraction) = self.xarm7.compute_cartesian_path(waypoints, 0.01, 0.0)
        self.xarm7.clear_pose_targets()
        self.ExecutePlan(traj)


def main():
    global shelf_position
    shelf_position = dict()
    # rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)

    move = Motion()
    rate = rospy.Rate(5)
    move.GetShelfPosition()
    id = 1

    while not rospy.is_shutdown():
        print("\n1-go to shelf",
              "\n2-to start",
              "\n3-test rotation",
              "\n4-to marker",
              "\n5-get coords",
              "\n6-shelf move",)
        inp = input('?:')
        if inp == "1": move.xArm7ToShelf(id)
        elif inp == "2": move.xArm7ToStart()
        elif inp == "3": move.testRotation(id)
        elif inp == "4": move.xArm7ToMarker(shelf_position[1])
        elif inp == "5": move.getCoordinates(id)
        elif inp == "6": move.shelfMove(id)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

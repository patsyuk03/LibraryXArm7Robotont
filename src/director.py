import rospy
import sys
from std_msgs.msg import String
from library.srv import *

def main():
    rospy.init_node('director', anonymous=True)

    while not rospy.is_shutdown():
        inp = input('\n1-scan for the shelf\n2-use saved shelf position\n?:')

        if inp == '1':
            rospy.loginfo('Wait for service find shelf.')
            rospy.wait_for_service('find_shelf')
            try:
                find_shelf = rospy.ServiceProxy('find_shelf', FindShelf)
                rospy.loginfo('Waiting responce.')
                res = find_shelf("LOOK FOR SHELF")
                rospy.loginfo(res.res)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)

        elif inp == '2':
            rospy.loginfo('Wait for service main.')
            rospy.wait_for_service('main')
            try:
                find_shelf = rospy.ServiceProxy('main', Main)
                rospy.loginfo('Waiting responce.')
                res = find_shelf('MAIN PROGRAM')
                rospy.loginfo(res.res)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
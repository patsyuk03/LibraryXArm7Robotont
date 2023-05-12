#!/usr/bin/python3

####################################################################

# Node that posts the information about occupated cells in the box #

####################################################################

import rospy
from std_msgs.msg import Int16MultiArray
from library.srv import Robotont, RobotontRequest

def main():
    rospy.init_node('sections_occupied', anonymous=True)
    try:
        serv = rospy.ServiceProxy('robotont', Robotont)
        rospy.loginfo('SECTIONS: Waiting responce.')
        res = serv(RobotontRequest(req=[6, 7]))
        if res.success:
            rospy.loginfo("SECTIONS: finished successfully.")
    except rospy.ServiceException as e:
        print("SECTIONS: Service call failed: %s"%e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
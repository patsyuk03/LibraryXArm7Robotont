################################################################################

# Node that sends a fake reqest to the main program to start transpoting books #

################################################################################

import rospy
from kitting_station.srv import Robotont, RobotontRequest

def main():
    rospy.init_node('fake_request', anonymous=True)
    try:
        serv = rospy.ServiceProxy('robotont', Robotont)
        rospy.loginfo('SECTIONS: Waiting responce.')
        res = serv(RobotontRequest(req=[6, 7]))                 # list contains the ids of markers that are attached to the cells which contain the books
        if res.success:
            rospy.loginfo("SECTIONS: finished successfully.")
    except rospy.ServiceException as e:
        print("SECTIONS: Service call failed: %s"%e)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
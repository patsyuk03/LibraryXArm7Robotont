#!/usr/bin/python3

####################################################################

# Node that posts the information about occupated cells in the box #

####################################################################

import rospy
from std_msgs.msg import Int16MultiArray

def main():
    rospy.init_node('sections_occupied', anonymous=True)
    pub = rospy.Publisher('sections', Int16MultiArray, queue_size=1)
    rate = rospy.Rate(10)
    sections = Int16MultiArray()
    sections.data = [8, 4]
    while not rospy.is_shutdown():
        pub.publish(sections)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
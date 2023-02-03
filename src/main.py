import rospy
import sys
from std_msgs.msg import String


def main():
    rospy.init_node('director', anonymous=True)
    pub = rospy.Publisher('director', String, queue_size=1)
    inp = input('\n1-scan for the shelf\n2-use previous shelf position\n?:')
    pub.publish(inp)
    
        


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
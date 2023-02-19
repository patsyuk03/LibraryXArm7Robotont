import rospy
import sys
from std_msgs.msg import String

def ShelfCallback(data):
    global shelf_answer
    shelf_answer = data.data

def MainCallback(data):
    global main_answer
    main_answer = data.data

def main():
    global shelf_answer, main_answer

    shelf_answer = 'NONE'

    rospy.init_node('director', anonymous=True)
    pub = rospy.Publisher('director', String, queue_size=1)
    rospy.Subscriber("find_shelf", String, ShelfCallback)
    rospy.Subscriber("main", String, MainCallback)
    while not rospy.is_shutdown():
        inp = input('\n1-scan for the shelf\n2-use saved shelf position\n3-exit\n?:')
        if inp == '1':
            while not rospy.is_shutdown():
                if shelf_answer == "WAITING":
                    pub.publish('LOOK FOR SHELF')
                else:
                    rospy.loginfo(shelf_answer)
                    break
        elif inp == '2':
            pub.publish('MAIN PROGRAM')
            while not rospy.is_shutdown():
                if main_answer == "WAITING":
                    pub.publish('MAIN PROGRAM')
                else:
                    break

        elif inp == '3':
            pub.publish('EXIT')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
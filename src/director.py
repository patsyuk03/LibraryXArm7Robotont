#####################################################

# Central node that coordinates what procram to run #

#####################################################

import rospy
from std_srvs.srv import Trigger, TriggerRequest

def main():
    rospy.init_node('director', anonymous=True)

    while not rospy.is_shutdown():
        inp = input('\n1-scan for the shelf\n2-use saved shelf position\n3-stop\n?:')

        if inp == '1':
            rospy.loginfo('DIRECTOR: Wait for service find shelf.')
            rospy.wait_for_service('find_shelf')
            try:
                find_shelf = rospy.ServiceProxy('find_shelf', Trigger)
                rospy.loginfo('DIRECTOR: Waiting responce.')
                res = find_shelf(TriggerRequest())
                if res.success:
                    rospy.loginfo("DIRECTOR: finished successfully.")
            except rospy.ServiceException as e:
                print("DIRECTOR: Service call failed: %s"%e)

        elif inp == '2':
            rospy.loginfo('DIRECTOR: Wait for service main.')
            rospy.wait_for_service('main')
            try:
                find_shelf = rospy.ServiceProxy('main', Trigger)
                rospy.loginfo('DIRECTOR: Waiting responce.')
                res = find_shelf(TriggerRequest())
                if res.success:
                    rospy.loginfo("DIRECTOR: finished successfully.")
            except rospy.ServiceException as e:
                print("DIRECTOR: Service call failed: %s"%e)

        elif inp == '3':
            rospy.loginfo('DIRECTOR: Stop.')
            break


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
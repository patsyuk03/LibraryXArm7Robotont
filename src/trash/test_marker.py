#!/usr/bin/python3

#########################################

# Made to test stuff (not only markers) #

#########################################

import rospy
import yaml
from ar_track_alvar_msgs.msg import AlvarMarkers
import geometry_msgs.msg

def ShelfPosition(data):
    global shelf_positions, shelf_positions_list, marker_ids

    for marker in data.markers:
        if marker.id not in list(shelf_positions.keys()) and marker.id in marker_ids:
            shelf_positions[marker.id] = marker.pose.pose

            position_x = marker.pose.pose.position.x
            position_y = marker.pose.pose.position.y
            position_z = marker.pose.pose.position.z
            position = {'x':position_x, 'y':position_y, 'z':position_z}

            orientation_x = marker.pose.pose.orientation.x
            orientation_y = marker.pose.pose.orientation.y
            orientation_z = marker.pose.pose.orientation.z
            orientation_w = marker.pose.pose.orientation.w
            orientation = {'x':orientation_x, 'y':orientation_y, 'z':orientation_z, 'w':orientation_w}

            shelf_positions_list.append({marker.id:{'position':position, 'orientation':orientation}})


class testStuff(object):
    def __init__(self):
        rospy.init_node('test_stuff', anonymous=True)
    
    def readYAML(self):
        global shelf_positions
        with open('src/library/yaml/shelf_position.yaml') as file:
            data = yaml.load(file, Loader=yaml.FullLoader)
            for i in range(3):
                id = list(data[i].keys())[0]
                shelf_positions[id] = geometry_msgs.msg.Pose()
                shelf_positions[id].position.x = data[i][id]['position']['x']
                shelf_positions[id].position.y = data[i][id]['position']['y']
                shelf_positions[id].position.z = data[i][id]['position']['z']
                shelf_positions[id].orientation.x = data[i][id]['orientation']['x']
                shelf_positions[id].orientation.y = data[i][id]['orientation']['y']
                shelf_positions[id].orientation.z = data[i][id]['orientation']['z']
                shelf_positions[id].orientation.w = data[i][id]['orientation']['w']
            print(shelf_positions)
    
    def writeYAML(self):
        global shelf_positions, shelf_positions_list
        print("write", shelf_positions)

        with open('src/library/yaml/shelf_position.yaml', 'w') as file:
            data = yaml.dump(shelf_positions_list, file)


def main():
    global shelf_positions, shelf_positions_list, marker_ids


    shelf_positions = dict()
    shelf_positions_list = []
    marker_ids = [1, 7, 0]

    rospy.Subscriber("/arm/ar_tf_marker", AlvarMarkers, ShelfPosition)
    test = testStuff()
    rate = rospy.Rate(5)
    test.readYAML()
    # while not rospy.is_shutdown():
    #     if len(shelf_positions)<3:
    #         rate.sleep()
    #     else: 
    #         test.writeYAML()
    #         break
    # test.readYAML()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
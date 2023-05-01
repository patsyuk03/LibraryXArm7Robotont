#!/usr/bin/python3

import rospy
import tf
import ar_track_alvar_msgs.msg
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped

def callback(data):
    if len(data.markers)!=0:
        tf_markers = ar_track_alvar_msgs.msg.AlvarMarkers()
        for marker in data.markers:
            tf_marker = marker
            marker.pose.header = marker.header
            marker.header.stamp = rospy.Time(0)
            p_in_base = listener.transformPose(link_name, marker.pose)
            tf_marker.pose = p_in_base
            tf_marker.header.frame_id = link_name
            tf_markers.markers.append(tf_marker)
        tf_markers.header = data.header
        tf_markers.header.frame_id = link_name
        pub_markers.publish(tf_markers)

def main():
    global listener, pub_markers, link_name
    rospy.init_node('transform_pose_marker', anonymous=True)
    link_name = rospy.get_param('link_name')
    # print(link_name)
    # rospy.loginfo("LINK NAME AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA: %s", link_name)
    listener = tf.TransformListener()
    pub_markers = rospy.Publisher('ar_tf_marker', AlvarMarkers, queue_size=1)
    rospy.loginfo("Subscribing to ar_pose_marker")
    rospy.Subscriber("ar_pose_marker", AlvarMarkers, callback)
    rospy.spin()
                    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
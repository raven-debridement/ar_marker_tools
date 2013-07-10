#!/usr/bin/env python
import roslib; roslib.load_manifest('marker_viz')
import rospy
from rll_utils.RvizUtils import place_arrow, draw_axes
from marker_detect.msg import *
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

pub = rospy.Publisher('visualization_marker', Marker)

def draw_markers(markers):
#    rospy.loginfo('Drawing markers ' + str(list(markers.ids)).\
#            replace('[', '{').replace(']', '}'))
    for id, pose in zip(markers.ids, markers.poses):
        ps = PoseStamped()
        ps.header = markers.header
        ps.pose = pose
        draw_axes(pub, id, 'ar_markers', ps, text=str(id))
        
if __name__ == '__main__':
    try:
        rospy.init_node('marker_viz_server')
        detector_topic = rospy.get_param('~marker_topic', 'marker_detector')
        sub = rospy.Subscriber(detector_topic, MarkerInfos, draw_markers)
        rospy.loginfo('Started marker visualization server.')
        rospy.spin()
    except rospy.ROSInterruptException: pass

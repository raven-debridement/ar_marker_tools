#!/usr/bin/env python
import roslib

roslib.load_manifest('marker_detector_node')
import rospy
from rll_utils.RvizUtils import *
from rll_utils.TFUtils import *
from marker_detector_node.msg import *
from geometry_msgs.msg._PointStamped import PointStamped
import tf

pub = rospy.Publisher('visualization_marker', Marker)

markers = {}

def broadcast_frames(msg):
    trans = SimpleTransformListener()
    for id, pose in zip(msg.ids, msg.poses):
        pp = PoseStamped()
        pp.header.frame_id = msg.header.frame_id
        pp.pose = pose
        ps = trans.transformPose('odom_combined', pp)
        markers[id] = ps
        
def main():
    rospy.init_node('marker_frames')
    sub = rospy.Subscriber('marker_detector_basket', MarkerInfos, broadcast_frames)
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        for id, pose in markers.items():
            point = pose.pose.position
            q = pose.pose.orientation
            br.sendTransform((point.x, point.y, point.z), 
                             (q.x, q.y, q.z, q.w),
                 rospy.Time.now(), 'marker_%d' % id, pose.header.frame_id)
        rospy.sleep(rospy.Duration(0.1))
        
if __name__ == '__main__':
    main()

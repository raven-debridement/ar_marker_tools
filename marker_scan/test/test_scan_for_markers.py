#!/usr/bin/env python
import roslib; roslib.load_manifest('marker_scan')
import rospy
from marker_scan.srv import ScanForMarkers
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion

rospy.init_node('marker_scanner_test')
rospy.loginfo('Waiting for scan_for_markers service...')
scan_service = rospy.wait_for_service('scan_for_markers')

try:
    srv = rospy.ServiceProxy('scan_for_markers', ScanForMarkers)
    header = Header()
    #TODO: get new data with fixed frame
    header.frame_id = 'base_link'
    header.stamp = rospy.Time.now()
    detector_service = 'marker_detector'
    ids = [5]
    # don't care about quaternion here
    poses = [Pose(Point(0.641, 0.053, 1.240), Quaternion(0.0, 0.0, 0.0, 1.0))]
    marker_infos = srv(header, detector_service, ids, poses)
    print marker_infos
except rospy.ServiceException, e:
    rospy.logerr('Service call failed: %s' % e)

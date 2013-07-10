#!/usr/bin/env python
import roslib; roslib.load_manifest('marker_scan')
import rospy
from marker_scan.srv import ScanFOV
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Pose, Quaternion

rospy.init_node('marker_scanner_test')
rospy.loginfo('Waiting for scan_fov service...')
scan_service = rospy.wait_for_service('scan_fov')

try:
    srv = rospy.ServiceProxy('scan_fov', ScanFOV)
    marker_infos = srv('marker_detector', -30, 30, -10, 30, 10)
    print marker_infos
except rospy.ServiceException, e:
    rospy.logerr('Service call failed: %s' % e)

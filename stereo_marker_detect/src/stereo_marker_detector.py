#!/usr/bin/env python

import roslib; roslib.load_manifest('stereo_marker_detect')
import rospy
import tf
from geometry_msgs.msg import PointStamped, Pose, Quaternion
from marker_detect.msg import MarkerInfos, MarkerXYs
from sensor_msgs.msg import CameraInfo
import image_geometry
import Util
from visualization_msgs.msg import MarkerArray

'''
Simple package which listens to MarkerInfos messages from two marker detector
topics and uses stereo vision to compute a (hopefully #FIXME) more accurate
depth estimate. Largely based off of stereo_click package stereo_converter.py.
'''

'''
#TODO: Any way to improve pose estimates as well? Currently does averaging, not
sure if even preferable to individual.
'''

POS_DIFF_THRESHOLD = 0.003
ORI_DIFF_THRESHOLD = 0.004

def quaternion_avg(q1, q2):
    return Quaternion((q1.x+q2.x)/2.0, (q1.y+q2.y)/2.0, (q1.z+q2.z)/2.0,\
            (q1.w+q2.w)/2.0)

# tuple
def pos_diff(p1, p2):
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)**0.5

# tuple
def ori_diff(o1, o2):
    return ((o1[0] - o2[0])**2 + (o1[1] - o2[1])**2 + (o1[2] - o2[2])**2\
            + (o1[3] - o2[3])**2)**0.5

class StereoMarkerDetector(object):

    def __init__(self, l_marker_xy_topic, r_marker_xy_topic, l_cam_info_topic,\
            r_cam_info_topic, output_topic, output_frame, camera_frame):
        self.listener = tf.TransformListener()

        rviz_topic = "stereo_marker_array"
        self.rviz_pub = rospy.Publisher(rviz_topic, MarkerArray)
        
        # get camera infos
        self.l_cam_info = rospy.wait_for_message(l_cam_info_topic, CameraInfo)
        self.r_cam_info = rospy.wait_for_message(r_cam_info_topic, CameraInfo)

        # subscribe to individual marker detectors
        self.l_marker_sub = rospy.Subscriber(l_marker_xy_topic, MarkerXYs,\
                self.l_marker_cb)
        self.r_marker_sub = rospy.Subscriber(r_marker_xy_topic, MarkerXYs,\
                self.r_marker_cb)
        
        self.output_topic = output_topic
        self.output_frame = output_frame

        # marker xys, compute and publish when we have both right and left
        self.mxys1 = None
        self.mxys2 = None

        # keep track of transform to make sure cameras aren't moving when
        # computing stereo estimate
        #self.listener.waitForTransform('/odom_combined', camera_frame,\
        #        rospy.Time.now(), rospy.Duration(1.0))
        #self.cam_odom_pos, self.cam_odom_ori =\
        #        self.listener.lookupTransform('/odom_combined', camera_frame,\
        #        rospy.Time())
        self.locked = False

        self.pub = rospy.Publisher(output_topic, MarkerInfos)

    def l_marker_cb(self, marker_xys):
        if self.locked: return
        self.mxys1 = marker_xys
        self.stereo_marker_cb()

    def r_marker_cb(self, marker_xys):
        if self.locked: return
        self.mxys2 = marker_xys
        self.stereo_marker_cb()

    def refresh(self, cam_odom_pos, cam_odom_ori):
        self.locked = False
        self.mxys1 = None
        self.mxys2 = None
        self.cam_odom_pos = cam_odom_pos
        self.cam_odom_ori = cam_odom_ori

    def stereo_marker_cb(self):
        if self.mxys1 and self.mxys2:
            self.locked = True
            # check camera view hasn't shifted
            #self.listener.waitForTransform('/odom_combined', camera_frame,\
            #    rospy.Time.now(), rospy.Duration(1.0))
            #cam_odom_pos, cam_odom_ori =\
            #    self.listener.lookupTransform('/odom_combined', camera_frame,\
            #    rospy.Time())
            #if pos_diff(self.cam_odom_pos, cam_odom_pos) > POS_DIFF_THRESHOLD\
            #        or ori_diff(self.cam_odom_ori, cam_odom_ori) > \
            #        ORI_DIFF_THRESHOLD:
            #    self.refresh(cam_odom_pos, cam_odom_ori)
            #    return # don't publish

            # publish stereo marker infos
            marker_infos = self.convert_stereo_marker_xys(self.mxys1, self.mxys2)
            self.pub.publish(marker_infos)

            #self.refresh(cam_odom_pos, cam_odom_ori)
            self.locked = False

    def convert_stereo_marker_xys(self, mxys1, mxys2):
        pts = []
        oris = []
        ids = []

        for id in mxys1.ids:
            if id in mxys2.ids:
                index1 = mxys1.ids.index(id)
                index2 = mxys2.ids.index(id)
                pt1 = mxys1.xys[index1]
                pt2 = mxys2.xys[index2]
                ori1 = mxys1.oris[index1]
                ori2 = mxys2.oris[index2]
                pt = self.convert_stereo(pt1, pt2)
                pts.append(pt)
                ids.append(id)
                # average pose estimates
                oris.append(quaternion_avg(ori1, ori2))

        marker_infos = MarkerInfos()
        marker_infos.header.frame_id = self.output_frame
        rviz_array = MarkerArray()
        for i in range(0, len(ids)):
            marker_infos.ids.append(ids[i])
            p = Pose()
            p.position = pts[i]
            p.orientation = oris[i]
            rviz_array.markers.append(Util.createRvizMarker(p, self.output_frame))
            marker_infos.poses.append(p)
        self.rviz_pub.publish(rviz_array)

        return marker_infos

    def convert_stereo(self, p1, p2):
        if self.l_cam_info.P[3] == 0:
            left = p1
            right = p2
        else:
            left = p2
            right = p1
        u = left.x
        v = left.y
        disparity = left.x - right.x

        stereo_model = image_geometry.StereoCameraModel()
        stereo_model.fromCameraInfo(self.l_cam_info, self.r_cam_info)
        (x,y,z) = stereo_model.projectPixelTo3d((u,v), disparity)
        camera_pt = PointStamped()
        camera_pt.header.frame_id = self.l_cam_info.header.frame_id
        camera_pt.header.stamp = rospy.Time.now()
        camera_pt.point.x = x
        camera_pt.point.y = y
        camera_pt.point.z = z
        #self.listener.waitForTransform(self.output_frame, camera_pt.header.frame_id,\
        #        rospy.Time.now(),rospy.Duration(4.0))
        #output_point = self.listener.transformPoint(self.output_frame, camera_pt)

        #return output_point.point

        return camera_pt.point

if __name__ == '__main__':
    try:
        rospy.init_node('stereo_marker_detect')

        l_marker_xy_topic = rospy.get_param('~l_marker_xy_topic', 'l_marker_xys')
        r_marker_xy_topic = rospy.get_param('~r_marker_xy_topic', 'r_marker_xys')
        
        l_cam_info_topic = rospy.get_param('~l_cam_info_topic', '')
        r_cam_info_topic = rospy.get_param('~r_cam_info_topic', '')

        output_topic = rospy.get_param('~output_topic', 'stereo_marker_detector')
        output_frame = rospy.get_param('~output_frame', 'left_optical_frame')

        camera_frame = rospy.get_param('~camera_frame', 'left_optical_frame')
        
        detector = StereoMarkerDetector(l_marker_xy_topic, r_marker_xy_topic,\
                l_cam_info_topic, r_cam_info_topic, output_topic, output_frame,\
                camera_frame)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
import roslib; roslib.load_manifest('marker_scan')
import rospy
import tf
from marker_detect.msg import MarkerInfos
from marker_scan.srv import ScanForMarkers, ScanForMarkersResponse,\
        ScanFOV, ScanFOVResponse
from geometry_msgs.msg import PointStamped, QuaternionStamped
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal
import actionlib
from sensor_msgs.msg import JointState
from math import pi, atan, sqrt
from brett_pr2.pr2 import Head

MAX_SCAN_TRIES = 3

class MarkerScanner(object):

    def __init__(self):
        self.tf_listener = tf.TransformListener()
        #self.move_head_client = actionlib.SimpleActionClient(\
        #        'head_traj_controller/point_head_action', PointHeadAction)
        self.head = Head(self.tf_listener)
        self.scan_for_markers_service = \
                rospy.Service('scan_for_markers', ScanForMarkers, self.scan_for_markers)
        self.scan_fov_service = \
                rospy.Service('scan_fov', ScanFOV, self.scan_fov)
        rospy.loginfo('scan_for_markers service started.')

    def scan_for_markers(self, req):
        ids_left = set(req.ids)
        scan_poses = req.poses
        marker_dict = {}
        marker_header = None
        tries = 0
        while ids_left:
            for scan_pose in scan_poses:
                #TODO adjust base/torso height to look directly at marker
                # if necessary
                if not ids_left:
                    break
                print 'GOT HERE'
                mp = PointStamped(req.header, scan_pose.position)
                mp.header.stamp = rospy.Time.now()
                self.look_at(mp)
                #self.fix_offset(mp)
                marker_infos_partial = MarkerInfos()
                for i in range(0, 5): # keep looking at same area for a while
                    try:
                        markers = rospy.wait_for_message(req.detector_service,\
                                MarkerInfos, timeout=1.0)
                    except rospy.exceptions.ROSException:
                        continue
                    marker_header = markers.header
                    for id, pose in zip(markers.ids, markers.poses):
                        if id not in ids_left:
                            continue
                        rospy.loginfo('Detected marker %d.' % id)
                        ids_left.remove(id)
                        marker_dict[id] = pose
                    rospy.loginfo('Markers remaining: ' +\
                        str(list(ids_left)).replace('[', '{').replace(']', '}'))
            tries += 1
            if tries > MAX_SCAN_TRIES:
                rospy.logerr('Couldn\'t find markers: ' +\
                    str(list(ids_left)).replace('[', '{').replace(']', '}'))
                break
        marker_poses = []
        ids = []
        for id in marker_dict:
            ids.append(id)
            marker_poses.append(marker_dict[id])
        marker_header.stamp = rospy.Time.now()
        return ScanForMarkersResponse(MarkerInfos(marker_header, marker_poses, ids))

    def scan_fov(self, req):
        marker_dict = {}
        marker_header = None
        for pan in range(req.start_pan, req.end_pan, req.angle_step):
            for tilt in range(req.start_tilt, req.end_tilt, req.angle_step):
                self.angle_head(pan*pi/180, tilt*pi/180)
                try:
                    markers = rospy.wait_for_message(req.detector_service,\
                            MarkerInfos, timeout=1.0)
                except rospy.exceptions.ROSException:
                    continue
                marker_header = markers.header
                for id, pose in zip(markers.ids, markers.poses):
                    if id in marker_dict:
                        continue
                    rospy.loginfo('Detected marker %d.' % id)
                    marker_dict[id] = pose
        marker_poses = []
        ids = []
        for id in marker_dict:
            ids.append(id)
            marker_poses.append(marker_dict[id])
        return ScanFOVResponse(MarkerInfos(marker_header, marker_poses, ids))

    def look_at(self, point, pointing_frame='narrow_stereo_optical_frame'):
        '''
        self.move_head_client.wait_for_server()

        goal = PointHeadGoal()
        goal.target.header.frame_id = point.header.frame_id
        goal.target.point = point.point
        goal.pointing_frame = pointing_frame
        goal.max_velocity = 0.25
        goal.min_duration = rospy.Duration(0.5)
        self.move_head_client.send_goal_and_wait(goal)

        return self.move_head_client.get_result()
        '''
        self.head.look_at(point, pointing_frame)

    # I believe as of Diamondback PointHeadAction still does not work
    # properly (Google "fixed pointheadaction")
    '''
    def fix_offset(self, point, pointing_frame = 'narrow_stereo_optical_frame'):
        self.tf_listener.waitForTransform(point.header.frame_id,\
                pointing_frame, rospy.Time.now(),\
                rospy.Duration(5.0))
        pos, rot = self.tf_listener.lookupTransform(point.header.frame_id,\
                pointing_frame, rospy.Time())
        mp = point.point

        joint_states = rospy.wait_for_message('joint_states', JointState, 5.0)
        pan = joint_states.position[joint_states.name.index('head_pan_joint')]
        tilt = pi/2-atan(sqrt((pos[0]-mp.x)**2+(pos[1]-mp.y)**2)/(pos[2]-mp.z))

        #HARDCODED
        pan-=pi/24
        tilt+=pi/24

        print pan, tilt
        self.angle_head(pan, tilt, pointing_frame)
    '''

    # adapted pr2_simple_head_motions
    def angle_head(self, pan, tilt, pointing_frame='double_stereo_link'):
        roll = 0
        yaw = pan
        pitch = tilt
        (qx,qy,qz,qw) = tf.transformations.quaternion_from_euler(roll,pitch,yaw)
        final_quat = QuaternionStamped()
        final_quat.header.frame_id = "torso_lift_link"
        final_quat.header.stamp = rospy.Time.now()
        final_quat.quaternion.x = qx
        final_quat.quaternion.y = qy
        final_quat.quaternion.z = qz
        final_quat.quaternion.w = qw
        #Convert the angle to the current stereo frame
        self.tf_listener.waitForTransform(pointing_frame,"torso_lift_link",rospy.Time.now(),rospy.Duration(10.0))
        rel_quat = self.tf_listener.transformQuaternion(pointing_frame,final_quat)
        rel_quat_tuple = (rel_quat.quaternion.x,rel_quat.quaternion.y,rel_quat.quaternion.z,rel_quat.quaternion.w)
        #Get the point which corresponds to one meter ahead of the stereo frame
        straight_target_q = (1.0,0,0,0)
        #Rotate by the quaternion
        (x,y,z,trash) = tf.transformations.quaternion_multiply(rel_quat_tuple,tf.transformations.quaternion_multiply(straight_target_q,tf.transformations.quaternion_inverse(rel_quat_tuple)))
        new_target = PointStamped()
        new_target.header.stamp = rospy.Time.now()
        new_target.header.frame_id = pointing_frame
        new_target.point.x = x
        new_target.point.y = y
        new_target.point.z = z
        #Now put in torso_lift_link frame
        self.tf_listener.waitForTransform("torso_lift_link",pointing_frame,rospy.Time.now(),rospy.Duration(10.0))
        target = self.tf_listener.transformPoint("torso_lift_link",new_target)
        self.look_at(target)

if __name__ == '__main__':
    try:
        rospy.init_node('scan_for_markers_server')
        ms = MarkerScanner()
        rospy.spin()
    except rospy.ROSInterruptException: pass


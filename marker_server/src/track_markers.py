#!/usr/bin/env python
import roslib; roslib.load_manifest('marker_server')
from marker_detect.msg import MarkerInfos
import rospy, tf
from collections import defaultdict
from geometry_msgs.msg import *
import yaml
from marker_server.srv import GetMarkerData, GetMarkerDataResponse
import sys

class MarkerTracker(object):

    def __init__(self, frame, topic, marker_file = None, marker_update=True):
        self.frame = frame
        self.listener = tf.TransformListener()
        rospy.sleep(2.0) # wait for tf buffer
        self.id2pose = dict()
        if marker_file != None:
            self.load(marker_file)
        self.listener.waitForTransform('map', self.frame, rospy.Time(),\
                rospy.Duration(1.0))
        self.srv = rospy.Service('get_marker_data', GetMarkerData, self.get_marker_data)
        self.marker_update = marker_update
        if marker_update:
            self.ar_sub = rospy.Subscriber(topic, MarkerInfos, self.proc_msg)        

    def load(self, filename):
        rospy.loginfo('Loading marker data from %s' % filename)
        marker_data = yaml.load(open(filename, 'r').read())
        if marker_data == None:
            rospy.loginfo('0 markers loaded.')
            return
        rospy.loginfo('%d markers loaded.' % len(marker_data))
        for id in marker_data:
            self.id2pose[id] = marker_data[id]['pose']

    def get_marker_data(self,req):
        resp = GetMarkerDataResponse()
        resp.header.frame_id = self.frame
        resp.header.stamp = rospy.Time.now()
        for (id,pose) in self.id2pose.items():
            resp.ids.append(id)
            resp.poses.append(pose)
        return resp

    def add_obs(self,id,pose):
        #pvec = msg2arr(pose)
        #if id in self.id2pvec:
        #    self.id2pvec[id] = self.id2pvec[id]*self.decay + pvec*(1-self.decay)
        #else:
        #    self.id2pvec[id] = pvec
        self.id2pose[id] = pose

    def proc_msg(self,msg):
        for (id,pose) in zip(msg.ids,msg.poses):
            ps = PoseStamped()
            ps.header.frame_id = msg.header.frame_id
            ps.header.stamp = msg.header.stamp
            ps.pose = pose
            try:
                self.listener.waitForTransform(msg.header.frame_id, self.frame,\
                #        msg.header.stamp,rospy.Duration(1.0))
                        msg.header.stamp, rospy.Duration(3.0))
                newpose = self.listener.transformPose(self.frame, ps).pose
            except Exception as e:
                rospy.loginfo(str(e) + ' Exiting.')
                rospy.signal_shutdown(str(e))
            self.add_obs(id, newpose)            

    def save(self,outfile):
        if not self.marker_update:
            return
        outdict = {}
        for (id, pose) in self.id2pose.items():
            #avgloc = pvec[id][0:3]
            #avgori = pvec[id][4:7]
            #avgori /= np.linalg.norm(avgori)
            outdict[id] = dict(pose=pose, name='')
        with open(outfile,'w') as out:
            out.write(yaml.dump(outdict, default_flow_style=False))
            rospy.loginfo('Finished writing file. Edit name fields if you want.')

def shutdown_hook():
    mt.save(marker_file)

if __name__ == '__main__':
    try:
        rospy.init_node('marker_server')
        marker_topic = rospy.get_param('~marker_topic', 'marker_detector')
        marker_update = rospy.get_param('~marker_update', True)
        tracker_frame = rospy.get_param('~tracker_frame', 'map')
        marker_file = rospy.get_param('~marker_file', '$(find marker_server)/data/laundry_markers.yaml')
        mt = MarkerTracker(tracker_frame, marker_topic, marker_file, marker_update)
        rospy.on_shutdown(shutdown_hook)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


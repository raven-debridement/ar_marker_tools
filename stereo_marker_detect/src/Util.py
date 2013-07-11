from visualization_msgs.msg import Marker
import rospy

def createRvizMarker(pose, frame, id_):
    marker = Marker()
    marker.header.frame_id = frame
    marker.header.stamp = rospy.Time.now()
    marker.id = id_
    marker.lifetime = rospy.Duration.from_sec(1.0)
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.006
    marker.scale.y = 0.006
    marker.scale.z = 0.003
    marker.color.a = 1.0
    marker.color.g = 255
    marker.pose = pose
    return marker

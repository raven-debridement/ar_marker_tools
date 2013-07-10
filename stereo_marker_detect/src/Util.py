from visualization_msgs.msg import Marker

def createRvizMarker(pose, frame):
    marker = Marker()
    marker.header.frame_id = frame
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 0.006
    marker.scale.y = 0.006
    marker.scale.z = 0.003
    marker.color.a = 1.0
    marker.color.g = 255
    marker.pose = pose
    return marker

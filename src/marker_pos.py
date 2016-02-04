#!/usr/bin/env python
import rospy
from visualization_msgs.msg._MarkerArray import MarkerArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from app import util
from app.markerPos.anchorMarker import AnchorMarker




publisherPoseStamped = None
publisherMarkerArray = None
markers = None




def callback(data):
    global markers
    
    unknownMarkers = []
    notTrustworthyMarkers = []
    positionMatrix = None
    totalMatrix = None
    totalScore = 0
    
    for marker in data.markers:
        
        if markers.has_key(marker.id):
            
            anchor = markers.get(marker.id)
            
            if anchor.isTrustworthy():
                m = anchor.getCameraMatrixByMarker(marker) * anchor.score
                totalScore += anchor.score
                
                if(totalMatrix is None):
                    totalMatrix = m
                else:
                    totalMatrix += m
            else:
                rospy.logwarn("NO TRUSTWORTHY marker has been detected. marker id: "+str(marker.id))
                notTrustworthyMarkers.append(marker)
        else:
            rospy.logwarn("Unknown marker has been detected. marker id: "+str(marker.id))
            unknownMarkers.append(marker)
    #
    
    if(totalMatrix is not None and totalScore > 0):
    
        positionMatrix = totalMatrix * (1.0/totalScore)
    
        for marker in unknownMarkers:
            newAnchor = AnchorMarker(marker.id, determined=False)
            newAnchor.resetMarkerByCameraMatrixAndTransMarker(positionMatrix, marker)
            markers[marker.id] = newAnchor
        #
        
        
        for marker in notTrustworthyMarkers:
            markers.get(marker.id).resetMarkerByCameraMatrixAndTransMarker(positionMatrix, marker)
        #
        
        publishMatrix(positionMatrix)
        publishMarkers()
    else:
        rospy.logwarn("No marker is sight!")
#eof




def publishMarkers():
    pass
    # ==== TO DO ====
#eof




def publishMatrix(matrix):
    global publisherPoseStamped
    
    header = Header(0, rospy.rostime.get_rostime(), "world")
    pose = util.matrixToPose(matrix)
    
    publisherPoseStamped.publish(PoseStamped(header, pose))
#eof




def main(argv=None):
    global publisherPoseStamped, markers
    
    markers ={ 4 : AnchorMarker(4, [10,20,0], [0,0,0,1], score=100) }
    
    publisherPoseStamped = rospy.Publisher('itech_ros/marker_pose/pose', PoseStamped, queue_size=10)
    #publisherMarkerArray = rospy.Publisher('itech_ros/marker_pose/markers', MarkerArray, queue_size=10)
    
    rospy.init_node('itech_marker_pos', anonymous=True)
    util.logBeginningInfo("marker_pos")
    
    rospy.Subscriber("/camera/apriltags_marker", MarkerArray, callback)
    rospy.spin()
#eof

if __name__ == '__main__':
    main()
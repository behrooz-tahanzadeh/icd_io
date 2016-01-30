#!/usr/bin/env python
import rospy, numpy, socket, sys
from visualization_msgs.msg._MarkerArray import MarkerArray
from visualization_msgs.msg._Marker import Marker
from tf import transformations
from geometry_msgs.msg import PointStamped, Point, PoseStamped, Pose, Quaternion
from std_msgs.msg import Header




publisherPoseStamped = None
publisherMarkerArray = None
markers = None




class AnchorMarker:
    
    
    def __init__(self, id, p=[0,0,0], o=[0,0,0,1], determined=True, score=0):
        self.id = id
        self.position = p
        self.orientation = o
        self.determined = determined
        self.score = score
        self.sampleNum = 0
        self.setMatrix()
    #eof
    
    
    
    def setMatrix(self):
        pm = transformations.translation_matrix(self.position)
        om = transformations.quaternion_matrix(self.orientation)
        self.matrix = numpy.dot(pm, om)
        return self
    #eof
    
    
    
    def isTrustworthy(self):
        return self.score > 20
    #eof
    
    
    
    def trustMore(self, d=5):
        if self.score<100:
            self.score += d
            
        if self.score>100:
            self.score = 100
            
        return self
    #eof
    
    
    
    def trustLess(self, d=5):
        if self.score>0:
            self.score -= d
            
        if self.score<0:
            self.score = 0
            
        return self
    #eof
    
    
    
    def getCameraMatrixByMarker(self, marker):
        q = marker.pose.orientation
        qm = transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
            
        t = marker.pose.position
        tm = transformations.translation_matrix([t.x, t.y, t.z])
            
        r = numpy.dot(tm, qm)
        r = transformations.inverse_matrix(r)
        
        return numpy.dot(r, self.matrix)
    #eof
    
    
    
    def resetMarkerByCameraMatrixAndTransMarker(self, cameraMatrix, TransMarker):
        q = TransMarker.pose.orientation
        qm = transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
            
        t = TransMarker.pose.position
        tm = transformations.translation_matrix([t.x, t.y, t.z])
            
        TransMatrix = numpy.dot(tm, qm)
        
        newMat = numpy.dot(cameraMatrix, TransMatrix)
        
        newT = tuple(transformations.translation_from_matrix(newMat))
        newQ = tuple(transformations.quaternion_from_matrix(newMat))
        
        for i in range(len(self.position)):
            self.position[i] = ((self.position[i]*self.sampleNum)+newT[i]) / (self.sampleNum+1)
            
        for i in range(len(self.orientation)):
            self.orientation[i] = ((self.orientation[i]*self.sampleNum)+newQ[i]) / (self.sampleNum+1)
        
        self.setMatrix()
        self.sampleNum += 1
        self.trustMore()
        
        return self
    #eof
    
    
#eoc




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
    
    t = tuple(transformations.translation_from_matrix(matrix))
    q = tuple(transformations.quaternion_from_matrix(matrix))
    
    header = Header(0, rospy.rostime.get_rostime(), "world")
    pose = Pose(Point(t[0], t[1], t[2]), Quaternion(q[0], q[1], q[2], q[3]))
    
    publisherPoseStamped.publish(PoseStamped(header, pose))
#eof




def main(argv=None):
    global publisherPoseStamped, markers
    
    markers ={ 4 : AnchorMarker(4, [0,0,0], [0,0,0,1], score=100) }
    
    publisherPoseStamped = rospy.Publisher('itech_ros/marker_pose/pose', PoseStamped, queue_size=10)
    publisherMarkerArray = rospy.Publisher('itech_ros/marker_pose/markers', MarkerArray, queue_size=10)
    
    rospy.init_node('itech_marker_pos', anonymous=True)
    rospy.loginfo("========= itech_marker_pos =========")
    
    rospy.Subscriber("/camera/apriltags_marker", MarkerArray, callback)
    rospy.spin()
#eof

if __name__ == '__main__':
    main()
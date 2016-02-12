import rospy
from geometry_msgs.msg._PoseStamped import PoseStamped
from visualization_msgs.msg._MarkerArray import MarkerArray
from app import util
from std_msgs.msg import Header
import numpy
from geometry_msgs.msg._PoseArray import PoseArray
from app.markerPos.poseCorrection import poseCorrection




class PoseController:
	"""
	Getting camera position in space based on known position of markers.
	
	:subscribe:
		/camera/apriltags_marker
	:publish:
		marker_pos/pos
		marker_pos/markers
	"""
	
	
	
	
	def __init__(self, anchors):
		
		self.anchors = anchors
		
		self.poseStampedPublisher = rospy.Publisher(util.topicName("marker_pos", "pose"), PoseStamped, queue_size=10)
		self.poseCorrectedStampedPublisher = rospy.Publisher(util.topicName("marker_pos", "pose_corrected"), PoseStamped, queue_size=10)
		#mavros/vision_pose/pose
		
		self.markerArrayPublisher = rospy.Publisher(util.topicName("marker_pos", "markers"), PoseArray, queue_size=10)
		
		rospy.Subscriber("/camera/apriltags_marker", MarkerArray, self.markerCb)
		
		self.cameraMatrix = None
		
		self.poseCorrection = poseCorrection().rotate180AroundX().rotate90AroundZ()
	#eof
	
	
	
	def markerCb(self, data):
		if self.calPosition(data.markers):
			self.publish()
	#eof
	
	
	
	
	def calPosition(self, markerArray):
		
		matrices = []
		
		for marker in markerArray:
			
			if self.anchors.has_key(marker.id):
				
				if(self.isValidMarker(marker)):
					anchor = self.anchors.get(marker.id)
					mat = anchor.getCameraMatrixByMarker(marker)
					matrices.append(mat)
				else:
					rospy.logwarn("Invalid Marker. id: "+str(marker.id))
			else:
				rospy.logwarn("Unknown Marker. id: "+str(marker.id))
		#eo for
		
		if len(matrices)>0:
			self.cameraMatrix = sum(matrices)/len(matrices)
			return True
		else:
			return False
	#eof
	
	
	
	
	def publish(self):
		self.publishPose()
		self.publishPoseCorrected()
		self.publishMarkers()
	#eof
	
	
	
	def publishPoseCorrected(self):
		if self.cameraMatrix is not None:
			header = Header(0, rospy.rostime.get_rostime(), "world")
			cm = self.poseCorrection.run(self.cameraMatrix)
			pose = util.matrixToPose(cm)
			self.poseCorrectedStampedPublisher.publish(PoseStamped(header, pose))
	#eof
	
	
	
	def publishPose(self):
		if self.cameraMatrix is not None:
			header = Header(0, rospy.rostime.get_rostime(), "world")
			pose = util.matrixToPose(self.cameraMatrix)
			self.poseStampedPublisher.publish(PoseStamped(header, pose))
	#eof
	
	
	
	
	def publishMarkers(self):
		poseArr = []
		
		header = Header(0, rospy.rostime.get_rostime(), "world")
		
		for a in self.anchors:
			poseArr.append(self.anchors[a].getPose())
		
		self.markerArrayPublisher.publish(PoseArray(header, poseArr))
	#eof
	
	
	
	def isValidMarker(self, marker):
		m = util.markerTransformationMatrix(marker)
		i = numpy.identity(4)
		return not numpy.allclose(m, i)
#eoc
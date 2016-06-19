import rospy
from geometry_msgs.msg._PoseStamped import PoseStamped
from visualization_msgs.msg._MarkerArray import MarkerArray
from app import util
from std_msgs.msg import Header
import numpy
from geometry_msgs.msg._PoseArray import PoseArray
from app.core.model.poseModel import PoseModel




class PoseController:
	"""
	Getting camera position in space based on known position of markers.
	
	:subscribe:
		/camera/apriltags_marker
	:publish:
		marker_pose/pose
		marker_pose/markers
	"""
	
	
	
	
	def __init__(self, anchors):
		self.anchors = anchors
		
		self.poseStampedPublisher = rospy.Publisher(util.topicName("marker_pose", "pose"), PoseStamped, queue_size=10)
		self.markerArrayPublisher = rospy.Publisher(util.topicName("marker_pose", "markers"), PoseArray, queue_size=10)
		self.poseSepPublisher = rospy.Publisher(util.topicName("marker_pose", "pose_sep"), PoseArray, queue_size=10)
		
		rospy.Subscriber("cam2/camera/apriltags_marker", MarkerArray, self.markerCb)
		
		self.cameraPose = PoseModel()
		self.poses = []
	#eof
	
	
	
	def markerCb(self, data):
		if self.calPosition(data.markers):
			self.publish()
	#eof
	
	
	
	
	def calPosition(self, markerArray):
		matrices = []
		poses = []
		
		for marker in markerArray:
			
			if self.anchors.has_key(marker.id):
				
				if(self.isValidMarker(marker)):
					anchor = self.anchors.get(marker.id)
					mat = anchor.getCameraMatrixByMarker(marker)
					matrices.append(mat)
					
					poses.append(PoseModel(mat).getPose())
				else:
					rospy.logwarn("Invalid Marker. id: "+str(marker.id))
			else:
				rospy.logwarn("Unknown Marker. id: "+str(marker.id))
		#eo for
		
		if len(matrices)>0:
			self.cameraPose.setMatrix(sum(matrices)/len(matrices))
			self.poses = poses
			return True
		else:
			return False
	#eof
	
	
	
	
	def publish(self):
		self.publishPose()
		self.publishMarkers()
	#eof
	
	
	
	
	def publishPose(self):
		if self.cameraPose is not None:
			self.poseStampedPublisher.publish(self.cameraPose.getPoseStamped())
			header = Header(0, rospy.rostime.get_rostime(), "world")
			self.poseSepPublisher.publish(PoseArray(header, self.poses))
	#eof
	
	
	
	
	def publishMarkers(self):
		poseArr = []
		
		header = Header(0, rospy.rostime.get_rostime(), "world")
		
		for a in self.anchors:
			poseArr.append(self.anchors[a].pose.getPose())
		
		self.markerArrayPublisher.publish(PoseArray(header, poseArr))
	#eof
	
	
	
	def isValidMarker(self, marker):
		m = util.markerTransformationMatrix(marker)
		i = numpy.identity(4)
		return not numpy.allclose(m, i)
	#eof
#eoc
import rospy
from geometry_msgs.msg._PoseStamped import PoseStamped
from visualization_msgs.msg._MarkerArray import MarkerArray
from app import util




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
		self.markerArrayPublisher = rospy.Publisher(util.topicName("marker_pos", "markers"), MarkerArray, queue_size=10)
		
		rospy.Subscriber("/camera/apriltags_marker", MarkerArray, self.markerCb)
	#eof
	
	
	
	
	def markerCb(self, data):
		#to do
		self.publish()
	#eof
	
	
	
	
	def publish(self):
		pass
	#eof
#eoc
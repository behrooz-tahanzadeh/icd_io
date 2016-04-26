import copy
from interactive_markers.interactive_marker_server import *  # @UnusedWildImport
from interactive_markers.menu_handler import *  # @UnusedWildImport
from visualization_msgs.msg import *  # @UnusedWildImport
from geometry_msgs.msg._Point import Point  # @UnusedImport
from interactive_markers import menu_handler  # @UnusedImport
from app import util
from geometry_msgs.msg._PoseStamped import PoseStamped




class RvizController:
	
	
	
	def __init__(self):
		#self.posePublisher = rospy.Publisher(util.topicName("rviz_dest", "pose"), PoseStamped, queue_size=10)
		#self.targetPublisher = rospy.Publisher(util.topicName("rviz_dest", "target"), PoseStamped, queue_size=10)
		
		#self.posePublisher = rospy.Publisher("/itech_ros/marker_pose/pose_corrected", PoseStamped, queue_size=10)
		self.targetPublisher = rospy.Publisher("/itech_ros/mavros_offboard/set_destination", PoseStamped, queue_size=10)
		
		self.server = InteractiveMarkerServer("rviz_dest")
		
		#self.createNorthDecPln()
		self.createTargetPln()
	#eof
	
	
	
	def createNorthDecPln(self):
		self.northDecMrk = InteractiveMarker()
		self.northDecMrk.header.frame_id = "world"
		self.northDecMrk.name = "Pose"
		self.northDecMrk.description = "Pose"
		
		self.makeBoxControl(self.northDecMrk)
		
		control = InteractiveMarkerControl()
		control.orientation.w = 1
		control.orientation.x = 0
		control.orientation.y = 1
		control.orientation.z = 0
		
		control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
		self.northDecMrk.controls.append(copy.deepcopy(control))
		
		control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
		self.northDecMrk.controls.append(control)
		self.server.insert(self.northDecMrk, self.poseCb)
		self.server.applyChanges()
	#eof
	
	
	
	def createTargetPln(self):
		self.targetMrk = InteractiveMarker()
		self.targetMrk.header.frame_id = "world"
		self.targetMrk.name = "Target"
		self.targetMrk.description = "Target"
		
		self.makeBoxControl(self.targetMrk)
		
		control = InteractiveMarkerControl()
		control.orientation.w = 1
		control.orientation.x = 0
		control.orientation.y = 1
		control.orientation.z = 0
		
		control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
		self.targetMrk.controls.append(copy.deepcopy(control))
		
		control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
		self.targetMrk.controls.append(control)
		self.server.insert(self.targetMrk, self.targetCb)
		self.server.applyChanges()
	#eof
	
	
	
	def poseCb(self,feedback):
		pass
		#header = Header(0, rospy.rostime.get_rostime(), "world")
		#self.posePublisher.publish(PoseStamped(header, feedback.pose))
	#eof
	
	
	
	def targetCb(self,feedback):
		header = Header(0, rospy.rostime.get_rostime(), "world")
		self.targetPublisher.publish(PoseStamped(header, feedback.pose))
	#eof




	def makeBox(self):
		marker = Marker()
		
		marker.type = Marker.ARROW
		marker.scale.x = 0.6
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.r = 1
		marker.color.g = 0
		marker.color.b = 0
		marker.color.a = 0.9
		
		return marker
	#eof




	def makeBoxControl(self,intMarker):
		control =  InteractiveMarkerControl()
		control.always_visible = True
		control.markers.append(self.makeBox())
		
		intMarker.controls.append( control )
		
		return control
	#eof
#eoc
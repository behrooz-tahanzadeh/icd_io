import rospy
from geometry_msgs.msg._PoseStamped import PoseStamped
from std_msgs.msg import Header
from geometry_msgs.msg._TwistStamped import TwistStamped
from app import util
from app.node.mavrosOffboard.mavSafety import MavSafety
from mavros_msgs.msg._VFR_HUD import VFR_HUD
import math
from std_msgs.msg._String import String




class MavController():
	
	def __init__(self, mavModel):
		
		self.cmdVelPub = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
		self.bodyPub = rospy.Publisher(util.topicName("mavros_offboard", "body"), PoseStamped, queue_size=10)
		self.enuPub = rospy.Publisher(util.topicName("mavros_offboard", "enu"), PoseStamped, queue_size=10)
		self.destENUPub = rospy.Publisher(util.topicName("mavros_offboard", "dest_enu"), PoseStamped, queue_size=10)
		self.safetyStatusPub = rospy.Publisher(util.topicName("mavros_offboard", "safety_status"), String, queue_size=10)
		
		rospy.Subscriber("/itech_ros/marker_pose/pose", PoseStamped, self.cameraCb)
		rospy.Subscriber(util.topicName("mavros_offboard", "set_destination"), PoseStamped, self.destCb)
		rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, self.hudCb)
		rospy.Subscriber("/itech_ros/mavros_local_pose_fusion/local_enu", PoseStamped, self.localEnuFusionCb)
		
		self.mavModel = mavModel
		self.mavSafety = MavSafety(self.mavModel)
	#eof
	
	
	
	def publish(self):
		if(not self.mavSafety.isSafe()):
			rospy.logerr("Not Safe")
			self.mavModel.twist.reset()		
		
		
		header = Header(0, rospy.rostime.get_rostime(), "world")
		
		if(not self.mavModel.destENU.isNone()):
			self.destENUPub.publish(self.mavModel.destENU.getPoseStamped(header))
		
		self.cmdVelPub.publish(self.mavModel.twist.getTwistStamepd(header))
	#eof	
	
	
	
	def cameraCb(self, data):
		if(self.mavModel.camera.isNone()):
			rospy.logwarn("First Localization Data Arrived!")
		
		self.mavModel.camera.setPoseStamped(data)
		self.mavModel.runPoseCorrection().runCalculation()
		
		if(not self.mavModel.body.isNone()):
			self.bodyPub.publish(self.mavModel.body.getPoseStamped())
		
		if(not self.mavModel.enu.isNone()):
			self.enuPub.publish(self.mavModel.enu.getPoseStamped())
	#eof
	
	
	
	def destCb(self, data):
		if(self.mavModel.dest.isNone()):
			rospy.logwarn("First Destination Data Arrived!")
			
		self.mavModel.dest.setPoseStamped(data)
		self.mavModel.runCalculation()
	#eof
	
	
	
	def hudCb(self, data):
		if(self.mavModel.headingAngle is None):
			rospy.logwarn("Heading Data Arrived!")
			
		self.mavModel.headingAngle = data.heading*(math.pi/180)
	#eof
	
	def localEnuFusionCb(self, data):
		self.mavModel.localEnuPose.setPoseStamped(data)
		self.mavModel.runCalculation()
	
#eoc
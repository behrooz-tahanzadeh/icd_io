import rospy
from geometry_msgs.msg._PoseStamped import PoseStamped
from std_msgs.msg import Header
from geometry_msgs.msg._TwistStamped import TwistStamped
from app.mavrosOffboard.mavModel import MavModel
from app import util




class MavController():
	
	def __init__(self):
		self.cmdVelPublisher = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
		self.destInPosePublisher = rospy.Publisher(util.topicName("mavros_offboard", "dest_in_pose"), PoseStamped, queue_size=10)
		self.PoseNorthDecPublisher = rospy.Publisher(util.topicName("mavros_offboard", "north_dec"), PoseStamped, queue_size=10)
		
		
		rospy.Subscriber("/itech_ros/marker_pose/pose_corrected", PoseStamped, self.poseCb)
		#rospy.Subscriber(util.topicName("mavros_offboard", "set_destination"), PoseStamped, self.destinationCb)
		rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.mavPoseCb)
		
		
		
		self.model = MavModel(0.3)
		
		p = PoseStamped()
		p.pose.position.x = 2.45
		p.pose.position.y = -2
		p.pose.position.z = 1.50
		
		self.model.dest = p.pose
	#eof
	
	
	
	def publishNorthDec(self):
		header = Header(0, rospy.rostime.get_rostime(), "world")
		self.PoseNorthDecPublisher.publish(PoseStamped(header, self.model.poseNorthDec))
	#eof
	
	
	
	def publish(self):
		self.publishVelocity()
		self.publishDestInPose()
		self.publishNorthDec()
	#eof	
	
	
	
	def publishDestInPose(self):
		header = Header(0, rospy.rostime.get_rostime(), "world")
		self.destInPosePublisher.publish(PoseStamped(header, self.model.destInPose))
	#eof
	
	
	
	def publishVelocity(self):
		header = Header(0, rospy.rostime.get_rostime(), "world")
		self.cmdVelPublisher.publish(TwistStamped(header, self.model.cmdVel))
	#eof
	
	
	
	def poseCb(self, data):
		self.model.pose = data.pose
	#eof
	
	
	
	def destinationCb(self, data):
		self.model.dest = data.pose
	#eof
	
	
	
	def mavPoseCb(self, data):
		self.model.mavPose = data.pose
	#eof
	
#eoc
import rospy
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from geometry_msgs.msg._TwistStamped import TwistStamped
from geometry_msgs.msg._Twist import Twist




class MavController():
	def __init__(self):
		self.setPointPublisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
		self.setVelocityPublisher = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
	#eof
	
	
	def publish(self):
		self.publishVelocity()
	#eof
	
	
	
	
	def publishPoint(self):
		header = Header(0, rospy.rostime.get_rostime(), "world")
		pose = Pose()
	
		pose.position.x = 0
		pose.position.y = 0
		pose.position.z = 1
	
		self.setPointPublisher.publish(PoseStamped(header, pose))
	#eof
	
	
	
	
	def publishVelocity(self):
		header = Header(0, rospy.rostime.get_rostime(), "world")
		twist = Twist()
	
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0
		
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		
	
		self.setVelocityPublisher.publish(TwistStamped(header, twist))
	#eof		
#eoc
import rospy
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from geometry_msgs.msg._TwistStamped import TwistStamped
from geometry_msgs.msg._Twist import Twist
from std_msgs.msg import String
import math




class MavController():
	
	def __init__(self):
		self.setPointPublisher = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
		self.setVelocityPublisher = rospy.Publisher("mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=10)
		
		rospy.Subscriber("/itech_ros/topic_from_udp/data", String, self.callback)
		
		self.twist = Twist()
		
		self.twist.angular.x = 0
		self.twist.angular.y = 0
		self.twist.angular.z = 0
		
		self.twist.linear.x = 0
		self.twist.linear.y = 0
		self.twist.linear.z = 0
		
		self.pose = Pose()
	#eof
	
	
	
	
	def publish(self):
		self.publishVelocity()
	#eof
	
	
	
	
	def publishPoint(self):
		header = Header(0, rospy.rostime.get_rostime(), "world")
	
		self.setPointPublisher.publish(PoseStamped(header, self.pose))
	#eof
	
	
	
	
	def publishVelocity(self):
		header = Header(0, rospy.rostime.get_rostime(), "world")
		
		self.setVelocityPublisher.publish(TwistStamped(header, self.twist))
	#eof		
	
	
	
	
	def callback(self, data):
		vArr = data.data.split(',')
		
		self.twist.linear.x = self.limitVelocity(vArr[0])
		self.twist.linear.y = self.limitVelocity(vArr[1])
		self.twist.linear.z = self.limitVelocity(vArr[2])
		
		self.pose.position.x = self.limitPosition(vArr[0])
		self.pose.position.y = self.limitPosition(vArr[1])
		self.pose.position.z = self.limitPosition(vArr[2])
	#eof
	
	
	
		
	def limitVelocity(self, number):
		try:
			number = float(number)
				
			if(math.fabs(number)>0.3):
				if(number < 0):
					return -0.3
				else:
					return 0.3
			else:
				return number
		except:
			rospy.logwarn("Invalid string")
	#eof
	
	
	
		
	def limitPosition(self, number):
		number = float(number)
			
		if(math.fabs(number)>1.5):
			if(number < 0):
				return -1.5
			else:
				return 1.5
		else:
			return number
	#eof
	
#eoc
from geometry_msgs.msg._Twist import Twist
from geometry_msgs.msg._TwistStamped import TwistStamped
from std_msgs.msg import Header
import rospy




class MavTwist:
	
	def __init__(self, mavModel):
		self.mavModel = mavModel
		self.twist = Twist()
	#eof
	
	
	
	def reset(self):
		self.twist.angular.x = 0
		self.twist.angular.y = 0
		self.twist.angular.z = 0
		
		self.twist.linear.x = 0
		self.twist.linear.y = 0
		self.twist.linear.z = 0
	#eof
	
	
	
	def getTwistStamepd(self, header = None):
		if header is None:
			header = Header(0, rospy.rostime.get_rostime(), "world")
		
		return TwistStamped(header, self.twist)
	#eof
	
	
	
	def calAngular(self):

		br = self.mavModel.body.getRotationAroundZ()
		dr = self.mavModel.dest.getRotationAroundZ()
		
		self.twist.angular.z = (dr-br)*self.mavModel.maxAngularSpeed

		return self
	#eof
	
	
	def calLinear(self):
		d = self.mavModel.destENU.getPositionDistance()
		
		if(d>self.mavModel.maxLinearSpeed):
			d = self.mavModel.maxLinearSpeed
			t = self.mavModel.destENU.getUnitTranslationVector()
			
			self.twist.linear.x = t[0]*d
			self.twist.linear.y = t[1]*d
			self.twist.linear.z = t[2]*d
		else:
			t = self.mavModel.destENU.getTranslation()
			
			self.twist.linear.x = t[0]*0.4
			self.twist.linear.y = t[1]*0.4
			self.twist.linear.z = t[2]*0.4
		
		return self
	#eof
#eoc

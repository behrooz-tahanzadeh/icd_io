import rospy
from mavros_msgs.msg import State




class MavState:
	
	def __init__(self, rate):
		rospy.Subscriber("/mavros/state", State, self.stateCb)
		
		self.currentState = State()
		self.rate = rate
	#eof
	
	
	
	def stateCb(self, data):
		self.currentState = data
	#eof
	
	
	
	def waitForFCUConnection(self):
	
		rospy.logwarn("Waiting for FCU connection...")
	
		while(not rospy.is_shutdown()):
			if(self.isConnected()):
				rospy.logwarn("FCU is connected...")
				return True
			
			self.rate.sleep()
		
		rospy.logwarn("ROS is shutdown")
		return False
	#eof
	
	
	
	def isConnected(self):
		return self.currentState.connected
	#eof
	
	
	
	def isArmed(self):
		return self.isConnected() and self.currentState.armed
	#eof
	
	
	
	def isOffboard(self):
		return self.isConnected() and self.currentState.mode == "OFFBOARD"
	#eof
#eoc
	
	
	
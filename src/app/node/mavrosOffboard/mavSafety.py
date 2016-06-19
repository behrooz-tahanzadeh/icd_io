import rospy




class MavSafety:
	
	def __init__(self, mavModel):
		self.mavModel = mavModel
	#eof
	
	def isSafe(self):
		#return self.isSafeObj(self.mavModel.camera)
		return True
	
	
	def isSafeObj(self, obj):
		if(not obj.isNone()):
			t = rospy.rostime.get_rostime()
			t = t.secs+(t.nsecs/1000000000.0)
		
		return (not obj.isNone()) and ((t - obj.timestamp) < 3)
	#eof
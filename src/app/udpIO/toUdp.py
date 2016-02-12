import rospy
import rostopic
from time import time
from socket import socket, AF_INET, SOCK_DGRAM
from geometry_msgs.msg._PoseStamped import PoseStamped
from std_msgs.msg._String import String
from app import util




class ToUdp:
	
	
	def __init__(self, ip, port, topicName, minInterval = None):
		
		self.udpArgs = (ip, port)
		self.socket = socket(AF_INET, SOCK_DGRAM)
		
		self.minInterval = minInterval
		self.lastTimeStamp = None
		
		self.topicName = topicName
		self.msgClass = rostopic.get_topic_class(self.topicName)[0]
	#eof
	
	
	
	
	def subscribe(self):
		if(self.isValidTopicName()):
			rospy.Subscriber(self.topicName, self.msgClass, self.callback)
			return True
		else:
			return False
	#eof
	
	
	
	
	def isValidTopicName(self):
		return self.msgClass is not None
	#eof
	
	
	
	
	def callback(self, data):
		
		if self.minInterval != None:
			ts = time()
			
			if self.lastTimestamp == None:
				self.lastTimestamp = ts
			elif (ts - self.lastTimestamp) <= self.minInterval:
				rospy.loginfo("skipped...")
				return
			else:
				self.lastTimestamp = ts
		#
		
		content = self.parseData(data)
		
		self.socket.sendto(content, self.udpArgs)
		rospy.loginfo("sent to: "+str(self.udpArgs))
	#eof
	
	
	
	
	def parseData(self, data):
		if self.msgClass is PoseStamped:
			m = util.markerTransformationMatrix(data).reshape(16)
			
			return ",".join(str(x) for x in m)
			
		elif self.msgClass is String:
			return data.data
		
		else:
			return str(data)
	#eof
#eoc
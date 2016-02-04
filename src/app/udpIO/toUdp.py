import rospy
import rostopic
from time import time
from socket import socket, AF_INET, SOCK_DGRAM



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
		
		content = str(data)
		self.socket.sendto(content, self.udpArgs)
	#eof
	
#eoc
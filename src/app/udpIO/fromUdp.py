import rospy
from app import util
from std_msgs.msg import String
from socket import socket, AF_INET, SOCK_DGRAM




class FromUdp:
	
	def __init__(self, ip, port):
		self.stringPublisher = rospy.Publisher(util.topicName("topic_from_udp", "data"), String, queue_size=10)
		
		self.udpArgs = (ip, port)
		self.socket = socket(AF_INET, SOCK_DGRAM)
		
		self.socket.bind(self.udpArgs)
	#eof
	
	
	
	
	def run(self):
		while not rospy.is_shutdown():
			data, addr = self.socket.recvfrom(1024)
			self.stringPublisher.publish(str(data))
	#eof
	
	
	
	def kill(self):
		self.socket.close()
	
#eoc
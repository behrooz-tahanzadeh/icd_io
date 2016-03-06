#!/usr/bin/env python
import rospy
from app import util
from app.udpIO.toUdp import ToUdp




def main(argv=None):
	rospy.init_node('itech_topic_to_udp', anonymous=True)
	util.logBeginningInfo('topic_to_udp')
	
	try:
		args = getArgs()
		
		toUdp = ToUdp(args[0], args[1], args[2], args[3])
		
		if(toUdp.subscribe()):
			rospy.spin()
		else:
			rospy.logwarn("Topic ["+args[2]+"] does not appear to be published yet.")
	
	except ValueError as e:
		rospy.logerr(e.message)
		return
#eof




def getArgs():
	topicName = rospy.get_param('~topic', None)
	ip = rospy.get_param('~ip', None)
	port = rospy.get_param('~port', None)
	minInterval = rospy.get_param('~int', None)
	
	if topicName is None:
		raise ValueError("Missong Arg: Topic name")
	
	if ip is None:
		raise ValueError("Missing Arg: IP address")
	
	if port is None:
		port = 8080
		rospy.logwarn("Default port: 8080")
	else:
		port = int(port)
	
	
	if minInterval is not None:
		minInterval = float(minInterval)
		
	return ip, port, topicName, minInterval
#eof




if __name__ == '__main__':
	main()
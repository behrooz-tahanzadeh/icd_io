#!/usr/bin/env python
import rospy, sys
from app import util
from app.udpIO.toUdp import ToUdp




def main(argv=None):
	rospy.init_node('itech_topic_to_udp', anonymous=True)
	util.logBeginningInfo('topic_to_udp')
	
	try:
		args = getArgs(argv)
		toUdp = ToUdp(args[0], args[1], args[2], args[3])
		
		if(toUdp.subscribe()):
			rospy.spin()
		else:
			rospy.logerr("Invalid Topic Name")
		
	except IndexError:
		rospy.logerr("Missing Arg: pass ip address and port")
		return
#eof




def getArgs(argv=None):
	
	if argv is None:
		argv=sys.argv
	
	argv = rospy.myargv(argv)
	
	topicName = argv[1]
	ip = argv[2]
	port = argv[3]
	
	if 4<len(argv):
		minInterval = float(argv[4])
	else:
		minInterval = None
	
	return ip, port, topicName, minInterval
#eof




if __name__ == '__main__':
	main()
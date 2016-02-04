#!/usr/bin/env python
import rospy, sys
from app import util
from app.udpIO.fromUdp import FromUdp




def main(argv=None):
	rospy.init_node('itech_topic_from_udp', anonymous=True)
	util.logBeginningInfo('topic_from_udp')
	
	
	try:
		ip,port = getArgs(argv)
		
		fromUdp = FromUdp(ip, port)
		
		try:
			fromUdp.run()
		except:
			fromUdp.kill()
		
	except IndexError:
		rospy.logerr("Missing Arg: pass ip address and port")
		return
#eof




def getArgs(argv=None):
	if argv is None:
		argv=sys.argv
	
	argv = rospy.myargv(argv)
	
	ip = argv[1]
	port = argv[2]
	
	return ip,int(port)
#eof




if __name__ == '__main__':
	main()
#!/usr/bin/env python
import rospy
from app import util
from app.udpIO.fromUdp import FromUdp




def main(argv=None):
	rospy.init_node('itech_topic_from_udp', anonymous=True)
	util.logBeginningInfo('topic_from_udp')
	
	
	try:
		ip,port = getArgs()
		
		fromUdp = FromUdp(ip, port)
		
		try:
			fromUdp.run()
		except:
			fromUdp.kill()
		
	except ValueError as e:
		rospy.logerr(e.message)
		return
#eof




def getArgs():
	ip = rospy.get_param('~ip', None)
	port = rospy.get_param('~port', None)
	
	if ip is None:
		ip=""
		rospy.logwarn("Listening to all of the recieved data")
	
	if port is None:
		port = 8080
		rospy.logwarn("Default port: 8080")
	else:
		port = int(port)
	
	return ip,port
#eof




if __name__ == '__main__':
	main()
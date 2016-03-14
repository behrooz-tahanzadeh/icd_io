#!/usr/bin/env python
import rospy
from app import util
from app.topicListSrv.topicListSrv import TopicListService




def main():
	
	rospy.init_node('itech_topic_list', anonymous=True)
	util.logBeginningInfo('topic_list')
	
	args = getArgs()
	
	srv = TopicListService(args[0], args[1])
	
	if(srv.run()):
		rospy.spin()
	else:
		rospy.logerr("Can not start service")
#eof



def getArgs():
	fs = rospy.get_param('~fs', None)
	rs = rospy.get_param('~rs', None)
	
	if fs is None:
		fs = ","
		rospy.logwarn("Field Separator: , ")
	
	if rs is None:
		rs = "#"
		rospy.logwarn("Record Separator: # ")
	
	return fs, rs
#eof


if __name__ == "__main__":
	main()
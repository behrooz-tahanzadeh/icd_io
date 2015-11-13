#!/usr/bin/env python
import rospy
import urllib
import urllib2
import rostopic
import sys




def callback(data):
    rospy.loginfo(str(data))




def listener(argv=None):
    rospy.init_node('itech_topic_io', anonymous=True)
    
    rospy.logwarn("\n********* ITECH_ROS_IO **********\n")
    
    try:
        topicName,msgClass = getArgs(argv)
        if msgClass is None:
            rospy.logerr("Invalid Topic Name: "+topicName)
            return
    except IndexError as e:
        rospy.logerr("Missing Arg: Please pass the name of topic")
        return
    
    rospy.Subscriber(topicName, msgClass, callback)
    rospy.spin()
#eof




def getArgs(argv=None):
    if argv is None:
        argv=sys.argv
        
    argv = rospy.myargv(argv)
    
    topicName = argv[1]
    
    msgClass, realTopic, msgEval = rostopic.get_topic_class(topicName)
    
    return topicName,msgClass
#eof


if __name__ == '__main__':
    listener()

#!/usr/bin/env python
import rospy
import rostopic
import sys
import socket




def callback(data, args):
    content = str(data)
    rospy.loginfo(args)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(content, args)
    




def listener(argv=None):
    rospy.init_node('itech_topic_io', anonymous=True)
    
    rospy.logwarn("\n********* ITECH_ROS_IO **********\n")
    
    try:
        topicName,msgClass,ip,port = getArgs(argv)
        if msgClass is None:
            rospy.logerr("Invalid Topic Name: "+topicName)
            return
    except IndexError as e:
        rospy.logerr("Missing Arg: Please pass the name of topic, ip address and port")
        return
    
    rospy.Subscriber(topicName, msgClass, callback, (ip, int(port)))
    rospy.spin()
#eof




def getArgs(argv=None):
    if argv is None:
        argv=sys.argv
        
    argv = rospy.myargv(argv)
    
    rospy.loginfo(argv)
    
    topicName = argv[1]
    ip = argv[2]
    port = argv[3]
    
    msgClass, realTopic, msgEval = rostopic.get_topic_class(topicName)
    
    return topicName,msgClass,ip,port
#eof


if __name__ == '__main__':
    listener()

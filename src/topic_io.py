#!/usr/bin/env python
import rospy, rostopic, sys, socket, getopt
from time import time

interval = None
lastTimestamp = None


def callback(data, args):
    global interval, lastTimestamp
    
    
    if interval != None:
        ts = time()
        
        if lastTimestamp == None:
            lastTimestamp = ts
        elif (ts - lastTimestamp) <= interval:
            rospy.loginfo("skipped...")
            return
        else:
            lastTimestamp = ts
    #if
    
    content = str(data)
    rospy.loginfo("sending data to: "+str(args))
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(content, args)
    




def listener(argv=None):
    rospy.init_node('itech_topic_io', anonymous=True)
    
    rospy.logwarn("\n********* ITECH_ROS_IO **********")
    
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
    
    global interval
    
    if argv is None:
        argv=sys.argv
    
    argv = rospy.myargv(argv)
    
    topicName = argv[1]
    ip = argv[2]
    port = argv[3]
    
    if 4<len(argv):
        interval = float(argv[4])
        rospy.loginfo("Minimum interval between each message has been set...")
    else:
        interval = None
    
    msgClass, realTopic, msgEval = rostopic.get_topic_class(topicName)
    
    return topicName,msgClass,ip,port
#eof




if __name__ == '__main__':
    listener()
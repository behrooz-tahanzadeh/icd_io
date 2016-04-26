#!/usr/bin/env python
import rospy
from std_srvs.srv._Trigger import Trigger



def add_two_ints_client():
    rospy.wait_for_service('itech_topic_list')
    try:
        add_two_ints = rospy.ServiceProxy('itech_topic_list', Trigger)
        resp1 = add_two_ints()
        return resp1.message
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        
if __name__ == "__main__":
    print add_two_ints_client()
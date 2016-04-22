#!/usr/bin/env python
import rospy
from app import util
from mavros_msgs.msg import RCIn
from geometry_msgs.msg._Twist import Twist
from turtlesim.msg import Pose 
import time




publisher = None
x = None
y = None



def callback(data):
	global publisher
	
	t = Twist()
	
	if((x<1 or x>10) or (y<1 or y>10)): 
		time.sleep(5)
	
	t.angular.z = (1500-data.channels[0])/500.0
	t.linear.x = (data.channels[1]-1500)/100.0
	
	publisher.publish(t)
#eof
	




def poseCb(data):
	global x,y
	x = data.x
	y = data.y


if __name__ == '__main__':
	rospy.init_node('itech_rc_reader', anonymous=True)
	
	publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
	
	rospy.Subscriber("/mavros/rc/in", RCIn, callback)
	rospy.Subscriber("/turtle1/pose", Pose, poseCb)
	rospy.spin()
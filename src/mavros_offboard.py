#!/usr/bin/env python
import rospy
from app.mavrosOffboard.mavState import MavState
from app.mavrosOffboard.mavController import MavController
from app import util




def main(argv=None):
	#Node
	rospy.init_node('itech_mavros_offboard', anonymous=True)
	util.logBeginningInfo("mavros_offboard")
	
	rate = rospy.Rate(20)
	
	mavState = MavState(rate)
	mavController = MavController()
	
	if mavState.waitForFCUConnection():
		while not rospy.is_shutdown():
			mavController.publish()
			rate.sleep()
#eof

if __name__ == '__main__':
	main()
#!/usr/bin/env python
import rospy
from app import util
from app.node.mavrosLocalPoseFusion.controller import Controller




def main(argv=None):
	#Node
	rospy.init_node('itech_mavros_local_pose_fusion', anonymous=True)
	util.logBeginningInfo("mavros_local_pose_fusion")
	
	Controller()
	rospy.spin()
#eof




if __name__ == '__main__':
	main()
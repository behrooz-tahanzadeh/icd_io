#!/usr/bin/env python
import rospy
from app import util
from app.rvizDest.rvizController import RvizController




def main(argv=None):
	rospy.init_node("itech_rviz_dest")
	util.logBeginningInfo("rviz_dest")
	
	RvizController()
	rospy.spin()
#eoof

if __name__=="__main__":
	main()
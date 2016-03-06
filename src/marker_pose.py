#!/usr/bin/env python
import rospy
from app import util
from app.markerPose.anchorMarker import AnchorMarker
from app.markerPose.poseController import PoseController




def main(argv=None):
	dx = 2
	dy = -2
	
	anchors ={ 
			35 : AnchorMarker(35, [dx+0,	dy+0,		0], [0,0,0,1]),
			31 : AnchorMarker(31, [dx+0.75,	dy+0.50,	0], [0,0,0,1]),
			33 : AnchorMarker(33, [dx+0.75,	dy-0.50,	0], [0,0,0,1]),
			30 : AnchorMarker(30, [dx+1.50,	dy+0.50,	0], [0,0,0,1]),
			32 : AnchorMarker(32, [dx+1.50,	dy-0.50,	0], [0,0,0,1]),
			34 : AnchorMarker(34, [dx+2.25,	dy+0,		0], [0,0,0,1]),
			}
	
	try:
		rospy.init_node('itech_marker_pose', anonymous=True)
		
		util.logBeginningInfo("marker_pose")
	
		PoseController(anchors)
		rospy.spin()
	except:
		rospy.logfatal("run ros before running this package")
	
#eof

if __name__ == '__main__':
	main()
#!/usr/bin/env python
import rospy
from app import util
from app.markerPose.anchorMarker import AnchorMarker
from app.markerPose.poseController import PoseController
from __builtin__ import int




def main(argv=None):
	dx = 2
	dy = -2
	
	#TODO: adding dynamic marker defenition mechanism
	
	anchors ={ 
			30 : AnchorMarker(30, [dx+0.45,	dy+0.0,	0], [0,0,0,1]),
			31 : AnchorMarker(31, [dx+0.90,	dy+0.0,	0], [0,0,0,1]),
			32 : AnchorMarker(32, [dx+1.35,	dy+0.0,	0], [0,0,0,1]),
			33 : AnchorMarker(33, [dx+1.80,	dy+0.0,	0], [0,0,0,1]),
			34 : AnchorMarker(34, [dx+2.25,	dy+0.0,	0], [0,0,0,1]),
			35 : AnchorMarker(35, [dx+2.70,	dy+0.0,	0], [0,0,0,1]),
			}
	
	try:
		rospy.init_node('itech_marker_pose', anonymous=True)
		
		util.logBeginningInfo("marker_pose")
		
		args = getArgs()
		
		PoseController(anchors, args)
		rospy.spin()
	except:
		rospy.logfatal("run ros before running this package")
	
#eof




def getArgs():
	northDeclination = rospy.get_param('~northdec', None)
	
	if northDeclination is None:
		northDeclination = 0
		rospy.logwarn("North Declination: 0 ")
	else:
		northDeclination = int(northDeclination)
	
	return northDeclination
#eof




if __name__ == '__main__':
	main()
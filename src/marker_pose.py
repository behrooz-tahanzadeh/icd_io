#!/usr/bin/env python
import rospy
from app import util
from app.node.markerPose.anchorMarker import AnchorMarker
from app.node.markerPose.poseController import PoseController




def main(argv=None):
	rospy.init_node('itech_marker_pose', anonymous=True)
	util.logBeginningInfo("marker_pose")
	
	
	try:
		anchors = getArgs()
	except ValueError as e:
		rospy.logfatal(e.message)
		anchors = None
	
	
	if anchors is not None:
		try:
			PoseController(anchors)
			rospy.spin()
		except:
			rospy.logfatal("Node has been terminated.")
#eof




def getArgs():
	tags = rospy.get_param('~tags', None)
	
	if(tags is None):
		raise ValueError("No tag has been defined!")
	else:
		try:
			tags = eval(str(tags))
			anchors = {}
			
			for key in tags.keys():
				value = tags[key]
				anchors[key] = AnchorMarker(key, value[0], value[1])
				
			return anchors
		except:
			raise ValueError("Invalid tag description file")
#eof




if __name__ == '__main__':
	main()
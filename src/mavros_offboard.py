#!/usr/bin/env python
import rospy
from app.node.mavrosOffboard.mavState import MavState
from app.node.mavrosOffboard.mavController import MavController
from app import util
from app.node.mavrosOffboard.mavModel import MavModel
import numpy




def main(argv=None):
	#Node
	rospy.init_node('itech_mavros_offboard', anonymous=True)
	util.logBeginningInfo("mavros_offboard")
	
	rate = rospy.Rate(20)
	
	maxLinearSpeed, maxAngularSpeed, correctionMatrix = getArgs()
	
	mavState = MavState(rate)
	mavModel = MavModel(maxLinearSpeed, maxAngularSpeed, correctionMatrix)
	mavController = MavController(mavModel)
	
	if mavState.waitForFCUConnection():
		while not rospy.is_shutdown():
			mavController.publish()
			rate.sleep()
#eof




def getArgs():
	maxLinearSpeed = rospy.get_param('~maxLinearSpeed', None)
	maxAngularSpeed = rospy.get_param('~maxAngularSpeed', None)
	correctionMatrix = rospy.get_param('~correctionMatrix', None)
	
	if maxLinearSpeed is None:
		maxLinearSpeed = 0.3
		rospy.logwarn("Default Linear Max Speed: 0.3")
		
	if maxAngularSpeed is None:
		maxAngularSpeed = 0.02
		rospy.logwarn("Default Angular Max Speed: 0.3")
		
	if correctionMatrix is None:
		rospy.logwarn("Default Correction Matrix: No Correction!")
	else:
		correctionMatrix = numpy.matrix(eval("["+correctionMatrix+"]")).reshape(4,4)
	
	return maxLinearSpeed, maxAngularSpeed, correctionMatrix
#eof




if __name__ == '__main__':
	main()
#!/usr/bin/env python
import rospy
from app import util
from geometry_msgs.msg._PoseStamped import PoseStamped
from tf2_msgs.msg._TFMessage import TFMessage
from geometry_msgs.msg._Pose import Pose
from std_msgs.msg import Header




publisher = None




def callback(data):
	global publisher
	
	pose = Pose(data.transforms[0].transform.translation, data.transforms[0].transform.rotation)
	header = Header(0, rospy.rostime.get_rostime(), "world")
	
	publisher.publish(PoseStamped(header, pose))
#eof
	


def main():
	global publisher
	
	rospy.init_node('itech_tf_to_vision', anonymous=True)
	util.logBeginningInfo('tf_to_vision')
	
	publisher = rospy.Publisher('mavros/vision_pose/pose', PoseStamped, queue_size=10)
	
	rospy.Subscriber("/tf", TFMessage, callback)
	rospy.spin()

if __name__ == '__main__':
	main()
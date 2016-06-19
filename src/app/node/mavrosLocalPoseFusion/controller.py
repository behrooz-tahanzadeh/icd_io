from geometry_msgs.msg._PoseStamped import PoseStamped
import rospy
from app.core.model.poseModel import PoseModel
from app import util
from tf.transformations import quaternion_matrix, translation_matrix, euler_matrix
import numpy
import math
from geometry_msgs.msg._PointStamped import PointStamped
from std_msgs.msg import Header




class Controller:
	
	def __init__(self):
		rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.localCb)
		rospy.Subscriber("/itech_ros/mavros_offboard/enu", PoseStamped, self.enuCb)
		
		self.localEnuPub = rospy.Publisher(util.topicName("mavros_local_pose_fusion", "local_enu"), PoseStamped, queue_size=10)
		self.offsetPub = rospy.Publisher(util.topicName("mavros_local_pose_fusion", "offset"), PointStamped, queue_size=10)
		
		self.offset = None
		self.localPose = PoseModel()
		self.localEnuPose = PoseModel()
		
		#tick posee are the poses at the time that we get positioning data.
		self.enuTickPose = PoseModel()
		self.localTickPose = PoseModel()
		
		self.r = euler_matrix(0, 0, math.pi/-2)
	#eof
	
	
	
	def localCb(self, data):
		self.localPose.setPoseStamped(data)
		
		if(not (self.enuTickPose.isNone() or self.offset is None)):
			t = self.localPose.getTranslation()
			q = self.enuTickPose.getQuaternion()
			
			q = quaternion_matrix(q)
			t = translation_matrix(t)
		
			self.localEnuPose.setMatrix(numpy.dot(q,t))
			
			t = self.localEnuPose.getTranslation()
			
			t[0] -= self.offset[0]
			t[1] -= self.offset[1]
			t[2] -= self.offset[2]
			
			q = self.localEnuPose.getQuaternion()
			
			self.localEnuPose.setTranslationQuaternion(t, q)
			
			p = PointStamped()
			p.point.x = self.offset[0]
			p.point.y = self.offset[1]
			p.point.z = self.offset[2]
			
			p.header = Header(0, rospy.rostime.get_rostime(), "world")
			
			self.offsetPub.publish(p)
			
			self.localEnuPub.publish(self.localEnuPose.getPoseStamped())
		
	#eof
	
	
	
	def calculateOffset(self):
		if(not (self.enuTickPose.isNone() or self.localTickPose.isNone())):
			t = self.localTickPose.getTranslation()
			q = self.enuTickPose.getQuaternion()
			
			t = translation_matrix(t)
			q = quaternion_matrix(q)
			
			rp = PoseModel(numpy.dot(q,t))
			
			t1 = rp.getTranslation()
			t2 = self.enuTickPose.getTranslation()
			
			self.offset = [t1[0]-t2[0], t1[1]-t2[1], t1[2]-t2[2]]
			
			rospy.logwarn(self.offset)
	#eof
	
	
	
	def enuCb(self, data):
		self.enuTickPose.setPoseStamped(data)
		self.calculateOffset()
		self.localTickPose.setMatrix(self.localPose.getMatrix())
	#eof
#eoc
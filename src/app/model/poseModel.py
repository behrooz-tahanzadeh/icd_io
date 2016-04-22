from app import util
import rospy
from geometry_msgs.msg._PoseStamped import PoseStamped
from std_msgs.msg import Header
from tf.transformations import inverse_matrix




class PoseModel:
	
	def __init__(self, matrix=None):
		self.matrix = matrix
	#eof
	
	
	@property
	def pose(self):
		util.matrixToPose(self.matrix)
	#eof
	
	@pose.setter
	def pose(self, obj):
		self.matrix = util.poseToMatrix(obj)
	#eof
	
	
	@property
	def poseStamped(self):
		header = Header(0, rospy.rostime.get_rostime(), "world")
		return PoseStamped(header, self.pose)
	#eof
	
	@poseStamped.setter
	def poseStamped(self, obj):
		self.matrix = util.poseToMatrix(obj.pose)
	#eof
	
	
	
	@property
	def inverseMatrix(self):
		return inverse_matrix(self.matrix)
	#eof
	
	@inverseMatrix.setter
	def inverseMatrix(self, matrix):
		self.matrix = inverse_matrix(matrix)
	#eof
	
	
	
	def transform(self, matrix):
		r = util.transformPoseByMatrix(self.matrix, matrix);
		return PoseModel(r)
	#eof
#eoc
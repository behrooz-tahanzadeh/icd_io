from geometry_msgs.msg._Twist import Twist
from tf.transformations import inverse_matrix, euler_from_matrix,\
	rotation_matrix, quaternion_from_matrix, translation_from_matrix
from app import util
from math import sqrt
from geometry_msgs.msg._Pose import Pose
from geometry_msgs.msg._Point import Point
from geometry_msgs.msg._Quaternion import Quaternion



class MavModel(object):
	
	
	def __init__(self, maxSpeed):
		self.destMat = util.transformationMatrix([0,0,0], [0,0,0,1])
		self.poseMat = None
		
		self.destInPose = None
		self.destInPoseMat = None
		
		self.mavPoseMat = None
		
		self.mavPoseDec = 0
		self.poseDec = 0
		
		self.twist = Twist()
		
		self.resetVel()
		
		self.poseNorthDec = None
		self.poseNorthDecMat = None
		self.poseNorthDecInverseMat = None
		
		self.maxSpeed = maxSpeed
	#eof
	
	
	
	
	def resetVel(self):
		self.twist.angular.x = 0
		self.twist.angular.y = 0
		self.twist.angular.z = 0
		
		self.twist.linear.x = 0
		self.twist.linear.y = 0
		self.twist.linear.z = 0
	#eof
	
	
	
	@property
	def dest(self):
		pass
	
	@dest.setter
	def dest(self, dest):
		self.destMat = util.poseToMatrix(dest)
		
		self.runCalculation()
	#eof
	
	
	
	@property
	def pose(self):
		pass
	
	@pose.setter
	def pose(self, pose):
		self.poseMat = util.poseToMatrix(pose)
		self.poseDec = euler_from_matrix(self.poseMat, 'szyx')[0]
		
		self.runCalculation()
	#eof
	
	
	@property
	def mavPose(self):
		pass
	
	@mavPose.setter
	def mavPose(self, pose):
		self.mavPoseMat = util.poseToMatrix(pose)
		self.mavPoseDec = euler_from_matrix(self.mavPoseMat, 'szyx')[0]
		
		self.runCalculation()
	#eof
	
	
	
	def runCalculation(self):
		try:
			self.calNorthPose()
			self.calDestInPose()
		except:
			self.resetVel()
	#eof
	
	
	
	def calDestInPose(self):
		if(self.destMat is not None and self.poseNorthDecInverseMat is not None):
			self.destInPoseMat = util.transformPoseByMatrix(self.destMat, self.poseNorthDecInverseMat)
			self.destInPose = util.matrixToPose(self.destInPoseMat)
			
			x = self.destInPose.position.x
			y = self.destInPose.position.y
			z= self.destInPose.position.z
			
			l =  sqrt(x**2 + y**2 + z**2)
			
			if(l>0):
				self.twist.linear.x = (x/l)*self.maxSpeed
				self.twist.linear.y = (y/l)*self.maxSpeed
				self.twist.linear.z = (z/l)*self.maxSpeed
			else:
				self.resetVel()
		else:
			self.resetVel()
	#eof
	
	
	
	
	@property
	def cmdVel(self):
		return self.twist
	#eof
	
	
	def calNorthPose(self):
		qm = rotation_matrix(self.poseDec-self.mavPoseDec, (0,0,1))
		q = quaternion_from_matrix(qm)
		t = translation_from_matrix(self.poseMat)
			
		self.poseNorthDec = Pose(Point(t[0], t[1], t[2]), Quaternion(q[0], q[1], q[2], q[3]))
		self.poseNorthDecMat = util.transformationMatrix(t, q)
		
		self.poseNorthDecInverseMat = inverse_matrix(self.poseNorthDecMat)
#eoc
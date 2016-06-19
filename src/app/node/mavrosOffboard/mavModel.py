from app.core.model.poseModel import PoseModel
from tf.transformations import translation_from_matrix, rotation_matrix,\
	translation_matrix
from app.node.mavrosOffboard.mavTwist import MavTwist
import numpy



class MavModel(object):
	
	
	def __init__(self, maxLinearSpeed, maxAngularSpeed, correctionMatrix):
		self.dest = PoseModel() 	#Destination point in World.
		self.destENU = PoseModel()	#Destination in ENU frame.
		self.camera = PoseModel()	#Camera position in World.
		self.body = PoseModel()		#Body position in World. Y is aligned with Pixhawk front. Corrected position of camera.
		self.enu = PoseModel()		#ENU frame. Origin of this frame is body position. Y is aligned with geographical North
		self.localEnuPose = PoseModel()
		
		self.headingAngle = None	#angle from pixhawk front to North in CCW(right-hand). The number is represented in radians 
		
		self.maxLinearSpeed = maxLinearSpeed
		self.maxAngularSpeed = maxAngularSpeed 
		self.correctionMatrix = correctionMatrix
		
		self.twist = MavTwist(self)
	#eof
	
	
	
	def runPoseCorrection(self):
		self.body.setMatrix(self.camera.getMatrix())
		
		if(self.correctionMatrix is not None):
			self.body.transformByMatrix(self.correctionMatrix)
		
		return self
	#eof
	
	
	
	def runCalculation(self):
		if(self.calENU() and self.calDestENU()):
			self.twist.calAngular().calLinear()
		else:
			self.twist.reset()
		
		return self
	#eof
	
	
	
	def calDestENU(self):
		if(self.enu.isNone() or self.dest.isNone() or self.localEnuPose.isNone()):
			return False
		else:
			enuInverse = self.localEnuPose.getInverseMatrix()
			self.destENU.setMatrix(numpy.dot(enuInverse, self.dest.getMatrix()))
			return True
	#eof
	
	
	
	def calENU(self):
		if(self.body.isNone() or self.headingAngle is None):
			return False
		else:
			br = self.body.getRotationAroundZ()
			
			q = rotation_matrix(br+self.headingAngle, (0,0,1))
			
			t = translation_from_matrix(self.body.getMatrix())
			t = translation_matrix(t)
			
			self.enu.setMatrix(t).transformByMatrix(q)
			return True
#eoc
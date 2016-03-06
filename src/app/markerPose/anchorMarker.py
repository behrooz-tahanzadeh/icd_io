from tf.transformations import inverse_matrix
from app import util




class AnchorMarker:
	
	
	
	def __init__(self, mid, p=[0,0,0], o=[0,0,0,1], determined=True, score=0):
		self.id = mid
		self.position = p
		self.orientation = o
		self.determined = determined
		self.score = score
		self.sampleNum = 0
		
		self.matrix = util.transformationMatrix(self.position, self.orientation)
	#eof
	
	
	
	def getCameraMatrixByMarker(self, marker):
		r = util.markerTransformationMatrix(marker)
		r = inverse_matrix(r)
		
		return util.transformPoseByMatrix(r, self.matrix)
	#eof
	
	
	
	def isTrustworthy(self):
		return self.score > 20
	#eof
	
	
	
	def trustMore(self, d=5):
		if self.score<100:
			self.score += d
			
		if self.score>100:
			self.score = 100
			
		return self
	#eof
	
	
	
	def trustLess(self, d=5):
		if self.score>0:
			self.score -= d
			
		if self.score<0:
			self.score = 0
			
		return self
	#eof
	
	
	
	def resetMarkerByCameraMatrixAndTransMarker(self, cameraMatrix, transMarker):
		transMatrix = util.markerTransformationMatrix(transMarker)
		newMat = util.transformPoseByMatrix(cameraMatrix, transMatrix)
		
		self.matrix = ((self.matrix*self.sampleNum)+newMat) / (self.sampleNum+1)
		
		self.sampleNum += 1
		self.trustMore()
		
		return self
	#eof
	
	def getPose(self):
		return util.matrixToPose(self.matrix)
	
	
#eoc
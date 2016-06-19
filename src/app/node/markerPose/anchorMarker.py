from app import util
from app.core.model.poseModel import PoseModel




class AnchorMarker:
	
	
	
	def __init__(self, mid, p=[0,0,0], o=[0,0,0,1]):
		self.pose = PoseModel()
		self.pose.id = mid
		self.pose.setMatrix(util.transformationMatrix(p, o))
	#eof
	
	
	
	def getCameraMatrixByMarker(self, marker):
		
		r = PoseModel().setMarker(marker).getInverseMatrix()
		return self.pose.getTransformedMatrix(r)
	#eof
#eoc
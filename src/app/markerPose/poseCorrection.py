# -*- coding: utf-8 -*-
from tf.transformations import quaternion_matrix
from numpy import dot
from math import sqrt

"""
w		x		y		z		Description
1		0		0		0		Identity quaternion, no rotation
0		1		0		0		180° turn around X axis
0		0		1		0		180° turn around Y axis
0		0		0		1		180° turn around Z axis
√0.5	√0.5	0		0		90° rotation around X axis
√0.5	0		√0.5	0		90° rotation around Y axis
√0.5	0		0		√0.5	90° rotation around Z axis
√0.5	-√0.5	0		0		-90° rotation around X axis
√0.5	0		-√0.5	0		-90° rotation around Y axis
√0.5	0		0		-√0.5	-90° rotation around Z axis
"""


class poseCorrection():
	
	def __init__(self):
		self.rot_p180_x = quaternion_matrix([1,0,0,0])
		self.rot_p180_y = quaternion_matrix([0,1,0,0])
		self.rot_p180_z = quaternion_matrix([0,0,1,0])
		
		self.rot_p90_x = quaternion_matrix([sqrt(0.5),0,0,sqrt(0.5)])
		self.rot_p90_y = quaternion_matrix([0,sqrt(0.5),0,sqrt(0.5)])
		self.rot_p90_z = quaternion_matrix([0,0,sqrt(0.5),sqrt(0.5)])
		
		self.rot_n90_x = quaternion_matrix([-sqrt(0.5),0,0,sqrt(0.5)])
		self.rot_n90_y = quaternion_matrix([0,-sqrt(0.5),0,sqrt(0.5)])
		self.rot_n90_z = quaternion_matrix([0,0,-sqrt(0.5),sqrt(0.5)])
		
		self.operationArr = []
	#eof
	
	
	
	
	def run(self, matrix):
		if len(self.operationArr) == 0:
			return matrix
		else:
			for r in self.operationArr:
				matrix = dot(matrix, r)
			
			return matrix
	#eof
	
	
	
	
	def rotate180AroundX(self):
		self.operationArr.append(self.rot_p180_x)
		return self
	#eof
	
	
	
	
	def rotate90AroundZ(self):
		self.operationArr.append(self.rot_p90_z)
		return self
	#eof
#eof
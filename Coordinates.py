from numpy import *
from numpy.linalg import *
from scipy.linalg import expm as expM
from matplotlib import *
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d import Axes3D
import scipy as scipy
from Constants import *

def rigidFix( ref, dat ):
	"""
	Find rigid transforms that fix dat to reference configuation
	INPUT:
	dat -- N x M x D -- N samples of M points in D dimensions
	ref -- M x D -- reference configuration for the points
	OUTPUT:
	rot -- N x D x D -- N orthogonal transformations
	ctr -- N x D -- N translations    
	finds rot, ofs such that:
	dat[k,...] ~= dot( ref, rot[k,...] ) + ctr[k,newaxis,:]
	"""
	dat = asarray(dat)
	N,M,D = dat.shape
	ctr = mean( dat, axis=1 ) - mean( ref, axis=0 )[newaxis,:]
	bof = dat - ctr[:,newaxis,:]
	rot = zeros( (N,D,D), dtype=dat.dtype )
	for k in xrange(N):
		R,_,_,_ = scipy.linalg.lstsq( ref, bof[k,...] )
		U,_,V = svd(R)
		rot[k,...] = dot( U, V )
	return rot, ctr


def findHomography(xPoints, yPoints):
	# xPoints and yPoints are n by 3 arrays
	# where the i'th row of xPoints corresponds to yPoints
	# this will output a matrix H such that
	# y = A * x
	if (xPoints.shape != yPoints.shape):
		raise Exception("Array shapes do not match")

	A = zeros([xPoints.shape[0]*3, 9])
	yVec = zeros([xPoints.shape[0]*3, 1])
	for i in range(xPoints.shape[0]):
		idx = 3*i
		A[idx, 0:3] = xPoints[i, :]
		A[idx+1, 3:6] = xPoints[i, :]
		A[idx+2, 6:9] = xPoints[i, :]
		yVec[idx:idx+3] = array([yPoints[i, :]]).T

	hVec, _, _, _ = lstsq(A, yVec)
	H = hVec.reshape((3, 3))
	return H

class Coordinates:
	def __init__(self):
		paperWidth = 21 #cm
		paperLength = 29.7 # cm
		self.paperPoints = array([[0, 0, 1], [paperWidth, 0, 1], [paperWidth, paperLength, 1], [0, paperLength, 1]])
		self.H = None
		self.calibrated = False

	def isCalibrated(self):
		return self.calibrated

	def calibrate(self, realPoints, orientation):
		# realPoints is a 4 by 3 array
		# where each row is a point in the real world
		# that corresponds to a paper point


		self.orientation = orientation

		xCol = realPoints[:, 0]
		yCol = realPoints[:, 1]
		zCol = realPoints[:, 2]

		self.x_offset = average(xCol)
		self.y_offset = average(yCol)
		self.z_offset = average(zCol)

		self.calibrated = True

	def transformPaperToReal(self, paperPoints):
		if (self.orientation == PaperOrientation.HORIZONTAL):
			x = self.x_offset + (paperPoints[0] - paperLength/2)
			y = self.y_offset + (paperPoints[1] + paperWidth/2)
			z = self.z_offset
		elif (self.orientation == PaperOrientation.VERTICAL):
			x = self.x_offset
			y = self.y_offset + (paperPoints[0] + paperWidth/2)
			z = self.z_offset + (paperPoints[1] - paperLength/2)
		return [x, y, z]


	def transformRealToPaper(self, realPoints):
		pass

if (__name__ == "__main__"):

	l = 29.7
	w = 21

	paperPoints = array([[0, 0, 1], [w, 0, 1], [w, l, 1], [0, l, 1]])

	y_offset = 10;
	realPoints = array([[0, y_offset, 0], [w, y_offset, 0], [w, y_offset, l], [0, y_offset, l]])
	realPointsH = array([[0, 0, y_offset], [w, 0, y_offset], [w, l, y_offset], [0, l, y_offset]])
	# realPoints = array([[26, 11, -2], 
	# 	[25, -1, -2], 
	# 	[34, 1, -1], 
	# 	[37, 13, -1.7]])

	coordinates = Coordinates()
	coordinates.calibrate(realPointsH, PaperOrientation.HORIZONTAL)

	# print(coordinates.transformPaperToReal(paperPoints))
	# print(realPoints)

	for point in paperPoints:
		print(coordinates.transformPaperToReal(point))
	# print(coordinates.transformPaperToReal(paperPoints))
	print(realPointsH)

	# print(coordinates.transformRealToPaper(realPoints))
	# print(paperPoints)


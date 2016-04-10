from numpy import *
from numpy.linalg import *
from scipy.linalg import expm as expM
from matplotlib import *
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d import Axes3D


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

	def calibrate(self, realPoints):
		# realPoints is a 4 by 3 array
		# where each row is a point in the real world
		# that corresponds to a paper point
		self.H = findHomography(self.paperPoints, realPoints)
		self.H_inv = inv(self.H)

	def transformPaperToReal(self, paperPoints):
		realPoints = dot(self.H, paperPoints.T)
		realPoints[abs(realPoints) < 0.001] = 0
		return realPoints.T

	def transformRealToPaper(self, realPoints):
		paperPoints = dot(self.H_inv, realPoints.T)
		paperPoints[abs(paperPoints) < 0.001] = 0
		return paperPoints.T


# if (__name__ == "__main__"):

# 	l = 29.7
# 	w = 21

# 	paperPoints = array([[0, 0, 1], [w, 0, 1], [w, l, 1], [0, l, 1]])

# 	y_offset = 12;
# 	realPoints = array([[0, y_offset, 0], [w, y_offset, 0], [w, y_offset, l], [0, y_offset, l]]);

# 	coordinates = Coordinates()
# 	coordinates.calibrate(realPoints)
# 	print(coordinates.transformPaperToReal(paperPoints))
# 	print(realPoints)

# 	print(coordinates.transformRealToPaper(realPoints))
# 	print(paperPoints)



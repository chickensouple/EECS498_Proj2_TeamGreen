from joy import *
from math498 import *
from Constants import *
from numpy import *

class DrawSquarePlan(Plan):
	def __init__(self, app, *arg, **kw):
		Plan.__init__(self, app, *arg, **kw)

		self.orientation = None

		self.squareLength = 10.16

		self.extraDist = 1

		self.squarePoints = [[4, -6, 1], 
			[4, -6-self.squareLength, 1], 
			[4+self.squareLength, -6-self.squareLength, 1], 
			[4+self.squareLength, -6, 1]]


	def setOrientation(self, orientation):
		self.orientation = orientation

	def behavior(self):
		for i in range(len(self.squarePoints)):
			startPt = array(self.squarePoints[i])
			if (i == len(self.squarePoints) - 1):
				endPt = array(self.squarePoints[0])
			else:
				endPt = array(self.squarePoints[i+1])


			direction = endPt - startPt
			direction = direction * self.extraDist / linalg.norm(direction)

			startPt -= direction
			endPt += direction

			while (self.app.drawLinePlan.isRunning()):
				yield self.forDuration(0.01)


			print("Start pt: " + str(startPt))
			print("Start pt(3d): " + str(self.app.coordinates.transformPaperToReal(startPt)))
			print("End pt: " + str(endPt))
			print("End pt(3d): " + str(self.app.coordinates.transformPaperToReal(endPt)))

			self.app.drawLinePlan.setPoints(startPt, endPt, self.orientation)

			yield self.app.drawLinePlan

		yield



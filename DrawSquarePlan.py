from joy import *
from math498 import *
from Constants import *
from numpy import *

class DrawSquarePlan(Plan):
	def __init__(self, app, *arg, **kw):
		Plan.__init__(self, app, *arg, **kw)

		self.orientation = None

		self.squareLength = 8

		self.extraDist = 1

		self.squarePoints = [[6, -6, 1], 
			[6, -6-self.squareLength, 1], 
			[6+self.squareLength, -6-self.squareLength, 1], 
			[6+self.squareLength, -6, 1]]


	def setOrientation(self, orientation):
		self.orientation = orientation

	def behavior(self):
		for i in range(len(self.squarePoints)):
			startPt = array(self.squarePoints[i])
			if (i == len(self.squarePoints) - 1):
				endPt = array(self.squarePoints[0])
			else:
				endPt = array(self.squarePoints[i+1])

			if (self.orientation == PaperOrientation.VERTICAL):
				temp = startPt[1]
				startPt[1] = startPt[0]
				startPt[0] = temp

				temp = endPt[1]
				endPt[1] = endPt[0]
				endPt[0] = temp


			direction = endPt - startPt
			direction = direction * self.extraDist / linalg.norm(direction)

			startPt -= direction
			endPt += direction


			if (self.orientation == PaperOrientation.HORIZONTAL):
				if (i == 0):
					startPt[2] -= 0.7
				if (i == 1):
					endPt[2] += 2
				elif (i == 2):
					startPt[2] += 2
					endPt[2] += 2
				elif (i == 3):
					startPt[2] += 2

			while (self.app.drawLinePlan.isRunning()):
				yield self.forDuration(0.01)


			print("Start pt: " + str(startPt))
			print("Start pt(3d): " + str(self.app.coordinates.transformPaperToReal(startPt)))
			print("End pt: " + str(endPt))
			print("End pt(3d): " + str(self.app.coordinates.transformPaperToReal(endPt)))

			self.app.drawLinePlan.setPoints(startPt, endPt, self.orientation, False)

			yield self.app.drawLinePlan

		yield



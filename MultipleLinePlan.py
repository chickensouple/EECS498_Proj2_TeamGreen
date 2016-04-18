from joy import *
from math498 import *
from Constants import *
from numpy import *

class MultipleLinePlan(Plan):
	def __init__(self, app, *arg, **kw):
		Plan.__init__(self, app, *arg, **kw)
		self.startPoints = []
		self.endPoints = []
		self.orientation = None

	def setPoints(self, startPoints, endPoints, orientation):
		# start and end are 3d points
		self.startPoints = startPoints
		self.endPoints = endPoints
		self.orientation = orientation


	def behavior(self):
		while (len(self.startPoints) != 0):


			startPoint = self.startPoints[0]
			endPoint = self.endPoints[0]

			while (self.app.drawLinePlan.isRunning()):
				yield self.forDuration(0.01)


			
			self.app.drawLinePlan.setPoints(startPoint, endPoint, self.orientation, True)
			yield self.app.drawLinePlan

			self.startPoints.pop(0)
			self.endPoints.pop(0)

		yield


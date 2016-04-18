from joy import *
from math498 import *
from Constants import *
from numpy import *

class DrawLinePlan(Plan):
	def __init__(self, app, *arg, **kw):
		Plan.__init__(self, app, *arg, **kw)
		self.points = []

		self.lift_dist = 4
		self.press_dist = 1
		self.orientation = None

	def setPoints(self, start, end, orientation, liftup):
		# start and end are 3d points
		start3d = array(self.app.coordinates.transformPaperToReal(start))
		end3d = array(self.app.coordinates.transformPaperToReal(end))

		if (orientation == PaperOrientation.VERTICAL):
			liftVec = array([-self.lift_dist, 0, 0])
			pressVec = array([self.press_dist, 0, 0])
		elif (orientation == PaperOrientation.HORIZONTAL):
			liftVec = array([0, 0, self.lift_dist])
			pressVec = array([0, 0, -self.press_dist])
		else:
			print("Not a valid paper setup")
			return

		self.points = []

		if (liftup):
			self.points.append(start3d + liftVec)
		self.points.append(start3d + pressVec)
		self.points.append(end3d + pressVec)
		if (liftup):
			self.points.append(end3d + liftVec)
		self.orientation = orientation

	def behavior(self):
		if (self.points == None):
			print("No set positions")

		while (len(self.points) != 0):
			print("Move to (" + str(self.points[0][0]) + ", " +
							 str(self.points[0][1]) + ", " +
							 str(self.points[0][2]) )
			targetPoint = self.points[0]

			while (self.app.moveToPointPlan.isRunning()):
				yield self.forDuration(0.01)

			self.app.moveToPointPlan.setPaperOrientation(self.orientation)
			self.app.moveToPointPlan.setPoint(targetPoint)
			yield self.app.moveToPointPlan

			self.points.pop(0)




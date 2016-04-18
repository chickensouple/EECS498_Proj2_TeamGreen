from joy import *
from math498 import *
from common import *

# class PaperSetup:
# 	VERTICAL = 0
# 	HORIZONTAL = 1

# class DrawLinePlan(Plan):
# 	def __init(self, app, *arg, **kw):
# 		Plan.__init__(self, app, *arg, **kw)
# 		self.points = None

# 		self.lift_dist = 2
# 		self.press_dist = 1

# 	def setPoints(start, end, paperSetup):
# 		# start and end are 3d points
# 		start3d = array(self.app.coordinates.transformPaperToReal(start))
# 		end3d = array(self.app.coordinates.transformPaperToReal(end))

# 		if (paperSetup == PaperSetup.VERTICAL):
# 			liftVec = array([-self.lift_dist, 0, 0])
# 			pressVec = array([self.press_dist, 0, 0])
# 		elif (paperSetup == PaperSetup.HORIZONTAL):
# 			liftVec = array([0, 0, self.lift_dist])
# 			pressVec = array([0, 0, -self.press_dist])
# 		else:
# 			print("Not a valid paper setup")
# 			return

# 		self.points.append(start3d + liftVec)
# 		self.points.append(start3d + pressVec)
# 		self.points.append(end3d + pressVec)
# 		self.points.append(end3d + liftVec)

# 	def behavior(self):
# 		pass



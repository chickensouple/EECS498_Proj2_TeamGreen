from joy import *
from math498 import *
from numpy import *
from math import *

class Draw3dLinePlan(Plan):
	def __init(self, app, *arg, **kw):
		Plan.__init__(self, app, *arg, **kw)
		self.angles = None
		self.moveDist = 2 # cm

	def setPoints(start, end, end_effector_angle):
		# start and end are 3d points
		start3d = array(start)
		end3d = array(end)

		currAng = self.app.motors.get_angles()


		dist = linalg.norm(end3d - start3d)
		lineDir = end3d - start3d / dist

		numMoves = int(floor(dist / self.moveDist))


		startAngle = array(self.app.arm.inverseKinematics(start3d, end_effector_ang, currAng))
		endAngle = array(self.app.arm.inverseKinematics(end3d, end_effector_ang, currAng))

		angleDist = endAngle - startAngle
		angleMoveDist = angleDist * moveDist / dist

		self.angles.append(startAngle)
		for i in range(numMoves):
			self.angles.append(startAngle + i * angleMoveDist)

		self.angles.append(endAngle)

	def behavior(self):
		if (self.angles == None):
			yield
			return

		while (len(self.angles) != 0):
			if (self.moveToAnglesPlan.isRunning()):
				yield self.forDuration(0.05)
			angle = self.angles[0]
			self.angles.pop(0)
			self.moveToAnglesPlan.setAngles(angle)
			yield self.moveToAnglesPlan


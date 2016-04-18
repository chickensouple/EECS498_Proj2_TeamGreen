from joy import *
from math498 import *
from numpy import *

class PaperOrientation:
	VERTICAL = 0
	HORIZONTAL = 1

class MoveToPointPlan(Plan):
	def __init__(self, app, *arg, **kw):
		Plan.__init__(self, app, *arg, **kw)
		self.point = None
		self.paperOrientation = None
		self.motorAngles = None

	def setPaperOrientation(self, orientation):
		self.paperOrientation = orientation

	def setPoint(self, point):
		self.point = array(point)

		motorAngles = array(self.app.motors.get_angles())
		motorAnglesNoEffector = motorAngles[0:3]

		modelAngles = self.app.arm.convertMotorToModelAngles(motorAnglesNoEffector)

		currPos = array(self.app.arm.getTool(modelAngles))[0:3]

		targetModelAngles = modelAngles
		dist = linalg.norm(currPos - self.point)
		# TODO: add break if this goes on too long

		i = 0
		while (dist > 0.2):
			currPos = array(self.app.arm.getTool(targetModelAngles))[0:3]

			direction = self.point - currPos
			dirNorm = linalg.norm(direction)
			if (dirNorm > 6):
				direction /= linalg.norm(direction)
			else:
				scalarFactor = (14/8) * log(i+1) + 2
				direction /= 6
				i += 1

			Jt = self.app.arm.getToolJac(modelAngles)
			deltaAngles = dot(linalg.pinv(Jt)[:,:len(direction)],direction)
			targetModelAngles += deltaAngles

			# print("Curr Pos: " + str(currPos))
			# print("Target Model Angles: " + str(targetModelAngles))
			dist = linalg.norm(currPos - self.point)


		for i in range(len(targetModelAngles)):
			targetModelAngles[i] = angle_wrap_pi(targetModelAngles[i])


		self.targetMotorAngles = self.app.arm.convertModelToMotorAngles(targetModelAngles)


		if (self.paperOrientation == PaperOrientation.VERTICAL):
			self.targetMotorAngles.append(-self.targetMotorAngles[1] + pi/2)
		elif (self.paperOrientation == PaperOrientation.HORIZONTAL):
			self.targetMotorAngles.append(-self.targetMotorAngles[1])


		print("Target Motor Angles: " + str(self.targetMotorAngles))


	def behavior(self):
		if (self.targetMotorAngles == None):
			yield

		
		self.app.motors.set_angles(self.targetMotorAngles)
		


		yield







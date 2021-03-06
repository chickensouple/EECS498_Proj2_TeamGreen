from joy import *
from math498 import *
from numpy import *
from Constants import *


class MoveToPointPlan(Plan):
	def __init__(self, app, *arg, **kw):
		Plan.__init__(self, app, *arg, **kw)
		self.point = None
		self.paperOrientation = None
		self.motorAngles = None
		self.moveDist = 2
		self.angles = None
		self.maxSpeed = 1

	def setPaperOrientation(self, orientation):
		self.paperOrientation = orientation

	def setPoint(self, point):
		# call setOrientation before set point
		self.point = array(point)

		if (self.paperOrientation == PaperOrientation.VERTICAL):
			self.point[0] -= effectorHeight
		elif (self.paperOrientation == PaperOrientation.HORIZONTAL):
			self.point[2] += effectorHeight

		motorAngles = array(self.app.motors.get_angles())
		motorAnglesNoEffector = motorAngles[0:3]

		modelAngles = self.app.arm.convertMotorToModelAngles(motorAnglesNoEffector)

		currPos = array(self.app.arm.getTool(modelAngles))[0:3]

		targetModelAngles = modelAngles
		dist = linalg.norm(currPos - self.point)
		initialDist = dist

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

			Jt = self.app.arm.getToolJac(targetModelAngles)
			deltaAngles = dot(linalg.pinv(Jt)[:,:len(direction)],direction)
			targetModelAngles += deltaAngles

			# print("Curr Pos: " + str(currPos))
			# print("Target Model Angles: " + str(targetModelAngles))
			dist = linalg.norm(currPos - self.point)


		for i in range(len(targetModelAngles)):
			targetModelAngles[i] = angle_wrap_pi(targetModelAngles[i])


		self.targetMotorAngles = self.app.arm.convertModelToMotorAngles(targetModelAngles)


		if (self.paperOrientation == PaperOrientation.VERTICAL):
			self.targetMotorAngles.append(-self.targetMotorAngles[1] - pi/2)
		elif (self.paperOrientation == PaperOrientation.HORIZONTAL):
			self.targetMotorAngles.append(-self.targetMotorAngles[1])


		numSteps = initialDist / self.moveDist
		self.angles = []
		anglesDelta = (array(self.targetMotorAngles) - array(motorAngles)) / numSteps

		for i in range(int(numSteps)):
			self.angles.append(motorAngles + i * anglesDelta)
		self.angles.append(self.targetMotorAngles)



	def behavior(self):
		if (self.angles == None):
			print("No set position")
			yield

		# print("Start: " + str(self.angles))

		while (len(self.angles) != 0):
			targetAngles = array(self.angles[0])
			# print("Target Angles: " + str(targetAngles))

			motorAngles = array(self.app.motors.get_angles())

			angleDiff = targetAngles - motorAngles

			maxAngleDiff = max(angleDiff)
			if (maxAngleDiff == 0):
				self.app.motors.set_default_speeds()
			else:
				speeds = abs(angleDiff * self.maxSpeed / maxAngleDiff)
				self.app.motors.set_speeds(speeds)
				# print("Angle Diffs: " + str(angleDiff))
				# print("Speeds: " + str(speeds))
			self.app.motors.set_angles(targetAngles)

			while (linalg.norm(angleDiff) > 0.1):
				yield self.forDuration(0.01)
				motorAngles = array(self.app.motors.get_angles())
				angleDiff = targetAngles - motorAngles
		
			self.angles.pop(0)
		


		# print("Done")
		self.angles = None

		yield







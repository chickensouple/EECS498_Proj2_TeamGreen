from joy import *
from math498 import *
from numpy import *

class MoveToAnglesPlan(Plan):
	def __init__(self, app, *arg, **kw):
		Plan.__init__(self, app, *arg, **kw)
		self.anglesTarget = None

	def setAngles(self, angles):
		self.anglesTarget = array(angles)

	def behavior(self):
		currAngles = array(self.app.motors.get_angles())
		angleDeltas = abs(self.anglesTarget - currAngles)

		maxSpeed = 8
		maxDelta = max(angleDeltas)
		speeds = currAngles * maxSpeed / maxDelta

		dist = linalg.norm(currAngles, self.anglesTarget)
		while (dist < 0.05):
			yield self.forDuration(0.05)
			dist = linalg.norm(currAngles, self.anglesTarget)


if __name__ == "__main__":

	class Temp:
		def __init__(self):
			self.DEBUG = True
	temp = Temp()
	plan = MoveToAnglesPlan(temp)
	plan.setAngles([0, 0, 0, 0])
	plan.start()


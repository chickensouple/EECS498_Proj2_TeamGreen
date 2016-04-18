from numpy import *
from numpy.linalg import *
from scipy.linalg import expm as expM
from matplotlib import *
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d import Axes3D
from joy import *
from joy.decl import *

class Motors:
	def __init__(self, motors):
		self.motors = motors
		self.centidegToRad = pi / 18000
		self.radToCentideg = 18000 / pi
		for motor in self.motors:
			motor.set_mode(0)
			motor.set_speed(2)
			motor.go_slack()

		self.motorDirs = [1, 1, 1, -1]
		self.motorOffsets = [0, 0, 0, 0]

	def get_angles(self):
		angles = []
		for motor, motorDir, motorOffset in zip(self.motors, self.motorDirs, self.motorOffsets):
			angles.append(float(motor.get_pos()) * self.centidegToRad * motorDir - motorOffset)
		return angles

	def get_angle(self, idx):
		return self.motors[idx].get_pos() * self.centidegToRad * self.motorDir[idx] - self.motorOffsets[idx]

	def set_angles(self, angles):
		for motor, motorDir, motorOffset, angle in zip(self.motors, self.motorDirs, self.motorOffsets, angles):
			sentAngle = int(angle * self.radToCentideg * motorDir + motorOffset)
			motor.set_pos(sentAngle)

	def set_angle(self, idx, angle):
		self.motors[idx].set_pos(int((angle + self.motorOffsets[idx]) * self.radToCentideg * self.motorDirs[idx]))

	def go_slack(self):
		for motor in self.motors:
			motor.go_slack()


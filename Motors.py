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

		self.motorDirs = [1, -1, 1, 1]

	def get_angles(self):
		angles = []
		for motor, motorDir in (self.motors, self.motorDirs):
			angles.append(float(motor.get_pos()) * self.centidegToRad * motorDir)
		return angles

	def get_angle(self, idx):
		return self.motors[idx].get_pos() * self.centidegToRad

	def set_angles(self, angles):
		for angle, motor in zip(angles, self.motors):
			motor.set_pos(angle * self.radToCentideg)

	def set_angle(self, idx, angle):
		self.motors[idx].set_pos(angle * self.radToCentideg)

	def go_slack(self):
		for motor in self.motors:
			motor.go_slack()		


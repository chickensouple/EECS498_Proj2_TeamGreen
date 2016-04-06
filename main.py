from numpy import *
from numpy.linalg import *
from scipy.linalg import expm as expM
from matplotlib import *
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d import Axes3D
from arm import Arm
from joy import *
from joy.decl import *
from MovePlan import *
import pdb

# Turn on interactive mode in MatPlotLib
ion()

class MainApp(JoyApp):
	def __init__(self, *arg, **kw):
		JoyApp.__init__(self, *arg, **kw)
		self.arm = Arm();
		self.f = gcf()
		self.f.clf()
		self.ax = f.gca(projection='3d')

		self.motors = []
		self.motors.append(self.robot.at.Nx01)
		self.motors.append(self.robot.at.Nx01)
		self.motors.append(self.robot.at.Nx01)
		self.motors.append(self.robot.at.Nx01)
		self.motors.append(self.robot.at.Nx01)
		self.motors.append(self.robot.at.Nx01)

		self.movePlan = MovePlan(self)

		self.pos1 = None
		self.pos2 = None

	def onStart(self):
		pass

	def onEvent(self, evt):
		if evt.type == TIMEREVENT:
			return
		if evt.type != KEYDOWN:
			return

		if evt.key == K_q:
			self.pos1 = self.arm.getTool(self.get_arm_angles())
		elif evt.key == K_w:
			self.pos2 = self.arm.getTool(self.get_arm_angles())
		elif evt.key == K_e:
			if (self.pos1 == None):
				print("Pos1 not set")
				return
			self.movePlan.setPos(self.pos1)
			self.movePlan.start()
		elif evt.key == K_r:
			if (self.pos2 == None):
				print("Pos2 not set")
				return
			self.movePlan.setPos(self.pos2)
			self.movePlan.start()
		elif evt.key == K_z:
			for motor in self.motors:
				motor.go_slack()

	def set_arm_angles(self, angles):
		for angle, motor in zip(angles, self.motors):
			motor.set_pos(angle * 18000 / pi)

	def get_arm_angles(self):
		angles = []
		for motor in self.motors:
			angles.append(float(motor.get_pos()) * pi / 18000)






robot = {"count": 2, "port": dict(TYPE="tty", glob="/dev/ttyACM0", baudrate=115200)}
scr = {}

app = MainApp(robot=robot, scr=scr)
app.run()

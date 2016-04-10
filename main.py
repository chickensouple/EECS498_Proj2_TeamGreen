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
from Motors import *
from Coordinates import *
import pdb

# Turn on interactive mode in MatPlotLib
ion()

class MainApp(JoyApp):
	def __init__(self, *arg, **kw):
		JoyApp.__init__(self, *arg, **kw)
		self.arm = Arm();
		self.f = gcf()
		self.f.clf()
		self.ax = self.f.gca(projection='3d')

		motors = []
		motors.append(self.robot.at.Nx01)
		motors.append(self.robot.at.Nx55)
		motors.append(self.robot.at.Nx08)
		motors.append(self.robot.at.Nx06)
		self.motors = Motors[motors]

		self.movePlan = MovePlan(self)

		self.pos1 = None
		self.pos2 = None
		self.coordinates = Coordinates()
		self.calibrateIdx = -1
		self.calibrationPoints = []

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
		elif evt.key == K_c:
			if (self.calibrateIdx == -1):
				print("Started Calibration. Move arm to lower left of paper, then press \'c\'")
			elif (self.calibrateIdx == 0):
				print("Move arm to lower right of paper, then press \'c\'")
				self.calibrationPoints.append(self.arm.at(self.motor.get_angles()))
			elif (self.calibrateIdx == 1):
				print("Move arm to upper right of paper, then press \'c\'")
				self.calibrationPoints.append(self.arm.at(self.motor.get_angles()))
			elif (self.calibrateIdx == 2):
				print("Move arm to upper left of paper, then press \'c\'")
				self.calibrationPoints.append(self.arm.at(self.motor.get_angles()))
			elif (self.calibrateIdx == 3):
				print ("Finished Calibration")
				self.calibrationPoints.append(self.arm.at(self.motor.get_angles()))
				self.coordinates.calibrate(self.calibrationPoints)
				self.calibrationPoints = []
			self.calibrateIdx += 1
			if (self.calibrateIdx >= 4):
				self.calibrateIdx = -1



robot = {"count": 2, "port": dict(TYPE="tty", glob="/dev/ttyACM0", baudrate=115200)}
scr = {}

app = MainApp(robot=robot, scr=scr)
app.run()

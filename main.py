from numpy import *
from numpy.linalg import *
from scipy.linalg import expm as expM
from matplotlib import *
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d import Axes3D
from Arm import *
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
		cfg = dict()
		JoyApp.__init__(self, cfg=cfg, *arg, **kw)
		self.arm = Arm();
		self.f = gcf()
		self.f.clf()
		self.ax = self.f.gca(projection='3d')

		motors = []
		motors.append(self.robot.at.Nx02)
		motors.append(self.robot.at.Nx08)
		motors.append(self.robot.at.Nx06)
		motors.append(self.robot.at.Nx04)
		self.motors = Motors(motors)

		self.movePlan = MovePlan(self)

		self.angles1 = None
		self.angles2 = None
		self.coordinates = Coordinates()
		self.calibrateIdx = -1
		self.calibrationPoints = []

		self.pos3d = None

		self.timeForPlot = self.onceEvery(1.0/3.0)

	def onStart(self):
		pass

	def onEvent(self, evt):

		if self.timeForPlot():
			self.arm.plot(self.motors.get_angles(), self.ax)
			self.ax.set(xlim=[-50,50],ylim=[-50,50],zlim=[-50,50])
			draw()
			pause(0.001)


		if evt.type == TIMEREVENT:
			return
		if evt.type != KEYDOWN:
			return


		if evt.key == K_q:
			self.angles1 = self.motors.get_angles()
			print("Set Angles1")
		elif evt.key == K_w:
			self.angles2 = self.motors.get_angles()
			print("Set Angles2")
		elif evt.key == K_e:
			if (self.angles1 == None):
				print("Angles 1 not set, press q to set")
				return
			self.motors.set_angles(self.angles1)
		elif evt.key == K_r:
			if (self.angles1 == None):
				print("Angles 2 not set, press w to set")
				return
			self.motors.set_angles(self.angles2)


		elif evt.key == K_z:
			self.motors.go_slack()

		elif evt.key == K_p:
			print (array(self.motors.get_angles()) * 180 / pi)


		elif evt.key == K_c:
			temp = array(self.arm.forwardKinematics(self.motors.get_angles()))
			if (self.calibrateIdx == -1):
				print("Started Calibration. Move arm to lower left of paper, then press \'c\'")
			elif (self.calibrateIdx == 0):
				print("Move arm to lower right of paper, then press \'c\'")
				self.calibrationPoints.append(temp)
			elif (self.calibrateIdx == 1):
				print("Move arm to upper right of paper, then press \'c\'")
				self.calibrationPoints.append(temp)
			elif (self.calibrateIdx == 2):
				print("Move arm to upper left of paper, then press \'c\'")
				self.calibrationPoints.append(temp)
			elif (self.calibrateIdx == 3):
				print ("Finished Calibration")
				self.calibrationPoints.append(temp)
				self.coordinates.calibrate(self.calibrationPoints)
				self.arm.armPlot.setPaperPoints(self.calibrationPoints)

				self.calibrationPoints = []
			self.calibrateIdx += 1
			if (self.calibrateIdx >= 4):
				self.calibrateIdx = -1

		elif evt.key == K_a:
			if (self.coordinates.isCalibrated()):
				self.pos3d = self.coordinates.transformPaperToReal([paperWidth/2, paperLength/2, 1])
			else:
				self.pos3d = [30, 0, 10]
			angles = self.arm.inverseKinematics(self.pos3d, -pi/2, self.motors.get_angles())
			if (angles == None):
				print("Can't reach location")
				return
			self.motors.set_angles(angles)
		elif evt.key == K_UP:
			self.pos3d[0] += 2
			angles = self.arm.inverseKinematics(self.pos3d, -pi/2, self.motors.get_angles())
			if (angles == None):
				print("Can't reach location")
				return
			self.motors.set_angles(angles)
		elif evt.key == K_DOWN:
			self.pos3d[0] -= 2
			angles = self.arm.inverseKinematics(self.pos3d, -pi/2, self.motors.get_angles())
			if (angles == None):
				print("Can't reach location")
				return
			self.motors.set_angles(angles)
		elif evt.key == K_LEFT:
			self.pos3d[1] += 2
			angles = self.arm.inverseKinematics(self.pos3d, -pi/2, self.motors.get_angles())
			if (angles == None):
				print("Can't reach location")
				return
			self.motors.set_angles(angles)
		elif evt.key == K_RIGHT:
			self.pos3d[1] -= 2
			angles = self.arm.inverseKinematics(self.pos3d, -pi/2, self.motors.get_angles())
			if (angles == None):
				print("Can't reach location")
				return
			self.motors.set_angles(angles)
		elif evt.key == K_i:
			self.pos3d[2] += 2
			angles = self.arm.inverseKinematics(self.pos3d, -pi/2, self.motors.get_angles())
			if (angles == None):
				print("Can't reach location")
				return
			self.motors.set_angles(angles)
		elif evt.key == K_k:
			self.pos3d[2] -= 2
			angles = self.arm.inverseKinematics(self.pos3d, -pi/2, self.motors.get_angles())
			if (angles == None):
				print("Can't reach location")
				return
			self.motors.set_angles(angles)



robot = {"count": 4, "port": dict(TYPE="tty", glob="/dev/ttyACM0", baudrate=115200)}
scr = {}

app = MainApp(robot=robot, scr=scr)
app.run()

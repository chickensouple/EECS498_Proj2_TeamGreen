from numpy import *
from numpy.linalg import *
from scipy.linalg import expm as expM
from matplotlib import *
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d import Axes3D
from arm_old import *
from joy import *
from joy.decl import *
from Motors import *
from Coordinates import *
from MoveToPointPlan import *
from DrawLinePlan import *
from DrawSquarePlan import *
from MultipleLinePlan import *
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
		motors.append(self.robot.at.Nx04)
		motors.append(self.robot.at.Nx06)
		self.motors = Motors(motors)

		self.orientation = PaperOrientation.HORIZONTAL

		self.angles1 = None
		self.angles2 = None
		self.coordinates = Coordinates()
		self.calibrateIdx = -1
		self.calibrationPoints = []

		self.pos3d = None

		self.timeForPlot = self.onceEvery(1.0/3.0)

		self.moveToPointPlan = MoveToPointPlan(self)
		self.drawLinePlan = DrawLinePlan(self)
		self.drawSquarePlan = DrawSquarePlan(self)
		self.multipleLinePlan = MultipleLinePlan(self)
		self.moveToPointPlan.setPaperOrientation(self.orientation)

		self.startPoints = [[5, -10], [10, -15]]
		self.endPoints = [[18, -18], [1, -2]]

		self.lines = [ 
			[(18,10), (10,22)], [(18,35), (27,24)], [(19,10), (27,25)], [(18,35), (10,22)],
			[(29,37), (30,11)], [(42,36), (42,12)], [(29,12), (41,36)], [(14,46), (13,70)], 
			[(31,45), (20,58)], [(22,58), (32,70)], [(45,46), (33,54)], [(38,70), (49,61)], 
			[(34,55), (49,62)] ]




	def onStart(self):
		pass

	def moveToPos(self, pos):
		self.moveToPointPlan.setPoint(pos)
		self.moveToPointPlan.start()
		# angles = self.arm.inverseKinematics(pos, self.motors.get_angles())
		# if (angles == None):
		# 	print("Can't reach location")
		# 	return
		# self.motors.set_angles(angles)
		# print("Target Angles: " + str(array(angles) * 180/pi))

	def onEvent(self, evt):
		if self.timeForPlot():
			motorAngles = array(self.motors.get_angles())
			modelAngles = self.arm.convertMotorToModelAngles(motorAngles)

			self.arm.plotReal3D(modelAngles, self.ax)
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
			self.moveToAnglesPlan.setAngles(self.angles1)
			self.moveToAnglesPlan.run()

			# self.motors.set_angles(self.angles1)
		elif evt.key == K_r:
			if (self.angles1 == None):
				print("Angles 2 not set, press w to set")
				return
			# self.motors.set_angles(self.angles2)
			self.moveToAnglesPlan.setAngles(self.angles2)
			self.moveToAnglesPlan.run()

		elif evt.key == K_t:
			if (self.orientation == PaperOrientation.HORIZONTAL):
				self.orientation = PaperOrientation.VERTICAL
				print("VERTICAL")
			else:
				self.orientation = PaperOrientation.HORIZONTAL
				print("Horizontal")

		elif evt.key == K_l:
			startPoints = []
			endPoints = []
			for line in self.lines:
				startPoint = list(line[0])
				endPoint = list(line[1])


				temp = startPoint[0]
				startPoint[0] = startPoint[1]
				startPoint[1] = -temp


				temp = endPoint[0]
				endPoint[0] = endPoint[1]
				endPoint[1] = -temp

				startPoints.append(startPoint)
				endPoints.append(endPoint)


			self.multipleLinePlan.setPoints(startPoints, endPoints, self.orientation)
			self.multipleLinePlan.start()

		elif evt.key == K_z:
			self.motors.go_slack()

		elif evt.key == K_p:
			angles = self.motors.get_angles()
			print("Motor Angles: " + str(array(angles) * 180 / pi))
			modelAngles = self.arm.convertMotorToModelAngles(angles)

			print("Model Angles: " + str(array(modelAngles) * 180/pi))

			pos3d = self.arm.getTool(modelAngles)[0:3]
			pos3d[2] -= effectorHeight
			print("Pos: " + str(pos3d))


		elif evt.key == K_s:
			if (not self.coordinates.isCalibrated()):
				print("Not calibrated")
				return
			self.drawSquarePlan.setOrientation(self.orientation)
			self.drawSquarePlan.start()
			# self.drawLinePlan.setPoints([10, 0], [10, 10], self.orientation)
			# self.drawLinePlan.start()


		elif evt.key == K_c:
			modelAngles = self.arm.convertMotorToModelAngles(self.motors.get_angles())
			temp = array(self.arm.getTool(modelAngles))[0:3]
			if (self.orientation == PaperOrientation.HORIZONTAL):
				temp[2] -= effectorHeight
			elif (self.orientation == PaperOrientation.VERTICAL):
				temp[0] += effectorHeight

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
				self.coordinates.calibrate(array(self.calibrationPoints), self.orientation)
				self.arm.setPaperPoints(array(self.calibrationPoints))

				self.calibrationPoints = []
			self.calibrateIdx += 1
			if (self.calibrateIdx >= 4):
				self.calibrateIdx = -1

		elif evt.key == K_a:
			if (self.coordinates.isCalibrated()):
				self.pos3d = self.coordinates.transformPaperToReal([paperWidth/2, paperLength/2, 1])
			else:
				self.pos3d = [30, 0, 20]
			self.moveToPos(self.pos3d)
		elif evt.key == K_UP:
			self.pos3d[0] += 2
			self.moveToPos(self.pos3d)
		elif evt.key == K_DOWN:
			self.pos3d[0] -= 2
			self.moveToPos(self.pos3d)
		elif evt.key == K_LEFT:
			self.pos3d[1] += 2
			self.moveToPos(self.pos3d)
		elif evt.key == K_RIGHT:
			self.pos3d[1] -= 2
			self.moveToPos(self.pos3d)
		elif evt.key == K_i:
			self.pos3d[2] += 2
			self.moveToPos(self.pos3d)
		elif evt.key == K_k:
			self.pos3d[2] -= 2
			self.moveToPos(self.pos3d)




robot = {"count": 4, "port": dict(TYPE="tty", glob="/dev/ttyACM*", baudrate=115200)}
scr = {}

app = MainApp(robot=robot, scr=scr)
app.run()




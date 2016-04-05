from numpy import *
from numpy.linalg import *
from scipy.linalg import expm as expM
from matplotlib import *
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d import Axes3D
from arm import Arm
from joy import *
from joy.decl import *
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

	def onStart(self):
		pass

	def onEvent(self, evt):
		pass




robot = {"count": 2, "port": dict(TYPE="tty", glob="/dev/ttyACM0", baudrate=115200)}
scr = {}

app = MainApp(robot=robot, scr=scr)
app.run()

from numpy import *
from numpy.linalg import *
from scipy.linalg import expm as expM
from matplotlib import *
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d import Axes3D
import pdb



def generateRotationMatrix(angle, axis):
	c = cos(angle)
	s = sin(angle)
	if (axis == 0):
		return array([[1, 0, 0], [0, c, -s], [0, s, c]])
	elif (axis == 1):
		return array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
	elif (axis == 2):
		return array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class Joint:
	def __init__(self, axis, pos):
		# axis will be 0, 1, 2
		# mapping to x, y, z
		self.axis = axis
		# self.length = length
		self.pos = pos

class Arm:
	def __init__(self, dof):
		self.dof = dof
		self.joints = []

		# position of end effector with respect to last joint
		self.endEffectorPos = array([[20, 0, 0]]).T

		axes = [2, 1, 1, 1]
		lengths = [0, 0, 10, 10]

		for i in range(dof):
			self.joints.append(Joint(axes[i], [lengths[i], 0, 0]))

	def getTransform(self, jointNum, angle):
		# gets the transformation matrix from jointNum to jointNum-1
		# if jointNum is 0, it gets transform from first joint to global
		joint = self.joints[jointNum]
		R = generateRotationMatrix(angle, joint.axis).T
		t = array([joint.pos])
		T = concatenate([R, t.T], 1)
		T = concatenate([T, array([[0, 0, 0, 1]])], 0)
		return T

	def getEndEffectorPos(self, angles):
		l = concatenate([self.endEffectorPos, [[1]]], 0)
		for i in range(len(self.joints) - 1, -1, -1):
			T = self.getTransform(i, angles[i])
			l = dot(T, l)
		return l

	def getJointPositions(self, angles):
		newJoint = array([[0, 0, 0, 1]]).T
		joints = newJoint
		for i in range(len(self.joints) - 1, -1, -1):
			T = self.getTransform(i, angles[i])
			joints = dot(T, joints)
			if (i != 0):
				joints = concatenate([joints, newJoint], 1)
		return joints


ion()

arm = Arm(4)
f = gcf()
ax = f.gca(projection='3d')

startAng = array([0, 0, 0, 0])
endAng = array([1, 1, 0.5, -1.5])
currAng = startAng


timeTaken = 3
pauseTime = 0.1
scalar = 1 / (timeTaken / pauseTime)


i = 0;
while 1:
	jointPositions = arm.getJointPositions(currAng)
	positions = concatenate([arm.getEndEffectorPos(currAng), jointPositions], 1)
	x = positions[0, :]
	y = positions[1, :]
	z = positions[2, :]
	ax.plot(x, y, z, marker='o', color='m')

	ax.set_xlabel('X axis')
	ax.set_ylabel('Y axis')
	ax.set_zlabel('Z axis')
	draw()
	if (i < timeTaken / pauseTime):
		currAng = currAng + (endAng - startAng) * scalar
		i += 1
	pause(pauseTime)



print(arm.getJointPositions([0, 0, 0, 0]))
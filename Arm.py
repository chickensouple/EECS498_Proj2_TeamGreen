from math import *
from math498 import *
from numpy import *
from ArmPlot import *

class Arm:
	def __init__(self):
		self.lengths = [14., 26.5, 25., 2.]
		self.angleOffsets = [0, 0.48, -pi/2 - 0.48, 0]

		# offsets
		self.angle_lower_bounds = [-pi/2 + offset for offset in self.angleOffsets]
		self.angle_upper_bounds = [pi/2 + offset for offset in self.angleOffsets]

		# print("BOUNDS:")
		# print self.angle_lower_bounds
		# print self.angle_upper_bounds

		self.armPlot = ArmPlot([0, 0, 0, 0], self.lengths)

	def plot(self, angles, ax):
		kinematicAngles = self.armAnglesToKinematicAngles(angles)
		# print kinematicAngles
		self.armPlot.plotReal3D(kinematicAngles, ax)

	def armAnglesToKinematicAngles(self, armAngles):
		# recieves arm angles in radians
		angles = []
		for angle, offset in zip(armAngles, self.angleOffsets):
			newAngle = angle + offset
			angles.append(newAngle)
		return angles

	def kinematicAnglesToArmAngles(self, kinematicAngles):
		# recieves kinematic angles in radians 
		angles = []
		for angle, offset in zip(kinematicAngles, self.angleOffsets):
			newAngle = angle - offset
			angles.append(newAngle)
		return angles
			

	def forwardKinematics(self, armAngles):
		# angles is in arm angles
		angles = self.armAnglesToKinematicAngles(armAngles)

		r1 = self.lengths[1] * cos(angles[1]) + \
			self.lengths[2] * cos(angles[1] + angles[2]) + \
			self.lengths[3] * cos(angles[1] + angles[2] + angles[3])

		z = self.lengths[0] + \
			self.lengths[1] * sin(angles[1]) + \
			self.lengths[2] * sin(angles[1] + angles[2]) + \
			self.lengths[3] * sin(angles[1] + angles[2] + angles[3])

		x = r1 * cos(angles[0])
		y = r1 * sin(angles[0])
		return [x, y, z]


	def inverseKinematics(self, pos, vertical, curr_arm_angles):
		x = pos[0]
		y = pos[1]
		z = pos[2] - self.lengths[0]


		curr_angles = self.armAnglesToKinematicAngles(curr_arm_angles)
		# print("Inverse Kinematics: " + str(curr_angles))

		# changing into spherical coordinates
		r2 = sqrt(z*z + x*x + y*y)
		phi = asin(z / r2)
		theta = atan2(y, x)

		minCost = -1
		minCostSol = None
		for alpha2 in linspace(self.angle_lower_bounds[2], self.angle_upper_bounds[2], 500):

			A = -2*self.lengths[1]*self.lengths[3]*sin(alpha2)
			B = 2*self.lengths[2]*self.lengths[3] + 2*self.lengths[1]*self.lengths[3]*cos(alpha2)
			C = self.lengths[1]**2 + self.lengths[2]**2 + self.lengths[3]**2
			
			det = (r2**2 - C - 2*self.lengths[1]*self.lengths[2]*cos(alpha2)) / sqrt(A**2 + B**2)
			if (det < -1 or det > 1):
				continue


			sols = self.__inverseKinematicsHelper(alpha2, r2, phi, A, B, C)

			sol1 = [sols[0][0], alpha2, sols[0][1]]
			sol2 = [sols[1][0], alpha2, sols[1][1]]
			solutions = [sol1, sol2]
			for sol in solutions:
				fullSol = [theta, sol[0], sol[1], sol[2]]
				if (not self.__inverseKinematicsCheckValidity(fullSol, vertical)):
					continue
				cost = self.__inverseKinematicsCost(array(curr_angles)[1:], sol)
				# temp = [theta, sol[0], sol[1], sol[2]]
				# print temp
				# temp = self.kinematicAnglesToArmAngles(temp)
				# print self.forwardKinematics(temp)
				if (minCost == -1 or cost < minCost):
					minCost = cost
					minCostSol = sol

		if (minCostSol == None):
			return None
		kinematicSol = [theta, minCostSol[0], minCostSol[1], minCostSol[2]]
		armSol = self.kinematicAnglesToArmAngles(kinematicSol)
		return armSol


	def __inverseKinematicsCheckValidity(self, angles, vertical):
		for angle, lower, upper in zip(angles, self.angle_lower_bounds, self.angle_upper_bounds):
			wrappedAngle = angle_wrap_pi(angle)
			if (wrappedAngle < lower or wrappedAngle > upper):
				return False

		angleSum = angles[1] + angles[2] + angles[3]
		angleSum = angle_wrap_pi(angleSum)
		if (vertical):
			if (abs(angleSum - (0)) > 0.1):
				return False
				pass
		else:
			if (abs(angleSum - (-pi/2)) > 0.1):
				return False
				pass
		return True


	def __inverseKinematicsCost(self, curr_angles, angles):
		return linalg.norm(array(curr_angles) - array(angles))


	def __inverseKinematicsHelper(self, alpha2, r2, phi, A, B, C):
		alpha3 = asin((r2**2-C-2*self.lengths[1]*self.lengths[2]*cos(alpha2)) / (sqrt(A**2 + B**2))) - atan2(B, A)

		alpha3_2 = pi - alpha3 - 2*atan2(B, A)

		return [[self.__inverseKinematicsAlpha1(alpha2, alpha3, phi), alpha3], \
			[self.__inverseKinematicsAlpha1(alpha2, alpha3_2, phi), alpha3_2]]
		

	def __inverseKinematicsAlpha1(self, alpha2, alpha3, phi):
		num = self.lengths[2]*sin(alpha2) + self.lengths[3]*sin(alpha2+alpha3)
		den = self.lengths[1] + self.lengths[2]*cos(alpha2) + self.lengths[3]*cos(alpha2+alpha3)

		alpha1 = phi - atan2(num, den)
		return alpha1



if (__name__ == "__main__"):
	arm = Arm()
	# angles = [0, pi/4, -pi/4, -pi/2]
	# angles = [0, -0.1, 0, 0]
	# [x, y, z] = arm.forwardKinematics(angles)
	# print x, y, z

	sol = arm.inverseKinematics([0, 10, 0], False, [0, 0, 0, 0])
	print array(sol) * 180 / pi
	print arm.forwardKinematics(sol)


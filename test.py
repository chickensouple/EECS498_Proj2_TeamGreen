from math import *
from math498 import *
from numpy import *

class Arm:
	def __init__(self):
		self.d0 = 14.
		self.d1 = 26.5
		self.d2 = 25.
		self.d3 = 2.

		# offsets
		self.alpha1_lower = -pi
		self.alpha1_higher = pi
		self.alpha2_lower = -3*pi/2
		self.alpha2_higher = 0
		self.alpha3_lower = -pi/2
		self.alpha3_higher = pi/2

	def forwardKinematics(self, angles):
		r1 = self.d1 * cos(angles[1]) + \
			self.d2 * cos(angles[1] + angles[2]) + \
			self.d3 * cos(angles[1] + angles[2] + angles[3])

		z = self.d0 + \
			self.d1 * sin(angles[1]) + \
			self.d2 * sin(angles[1] + angles[2]) + \
			self.d3 * sin(angles[1] + angles[2] + angles[3])


		x = r1 * sin(angles[0])
		y = r1 * cos(angles[0])
		return [x, y, z]


	def inverseKinematics(self, pos, vertical, curr_angles):
		x = pos[0]
		y = pos[1]
		z = pos[2] - self.d0

		# changing into spherical coordinates
		r2 = sqrt(z*z + x*x + y*y)
		phi = asin(z / r2)
		theta = atan2(x, y)

		# print theta

		minCost = -1
		minCostSol = None
		for alpha2 in linspace(self.alpha2_lower, self.alpha2_higher, 500):

			A = -2*self.d1*self.d3*sin(alpha2)
			B = 2*self.d2*self.d3 + 2*self.d1*self.d3*cos(alpha2)
			C = self.d1**2 + self.d2**2 + self.d3**2
			
			det = (r2**2 - C - 2*self.d1*self.d2*cos(alpha2)) / sqrt(A**2 + B**2)
			if (det < -1 or det > 1):
				continue


			sols = self.__inverseKinematicsHelper(alpha2, r2, phi, A, B, C)

			sol1 = [sols[0][0], alpha2, sols[0][1]]
			sol2 = [sols[1][0], alpha2, sols[1][1]]
			solutions = [sol1, sol2]
			for sol in solutions:
				if (not self.__inverseKinematicsCheckValidity(sol, vertical)):
					continue
				cost = self.__inverseKinematicsCost(array(curr_angles)[1:], sol)
				if (minCost == -1 or cost < minCost):
					minCost = cost
					minCostSol = sol

		if (minCostSol == None):
			return None
		return [theta, minCostSol[0], minCostSol[1], minCostSol[2]]


	def __inverseKinematicsCheckValidity(self, angles, vertical):
		if (angles[0] < self.alpha1_lower or angles[0] > self.alpha1_higher):
			return False
		if (angles[1] < self.alpha2_lower or angles[1] > self.alpha2_higher):
			return False
		if (angles[2] < self.alpha3_lower or angles[2] > self.alpha3_higher):
			return False

		angleSum = angles[0] + angles[1] + angles[2]
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
		alpha3 = asin((r2**2-C-2*self.d1*self.d2*cos(alpha2)) / (sqrt(A**2 + B**2))) - atan2(B, A)

		alpha3_2 = pi - alpha3 - 2*atan2(B, A)

		return [[self.__inverseKinematicsAlpha1(alpha2, alpha3, phi), alpha3], \
			[self.__inverseKinematicsAlpha1(alpha2, alpha3_2, phi), alpha3_2]]
		

	def __inverseKinematicsAlpha1(self, alpha2, alpha3, phi):
		num = self.d2*sin(alpha2) + self.d3*sin(alpha2+alpha3)
		den = self.d1 + self.d2*cos(alpha2) + self.d3*cos(alpha2+alpha3)

		alpha1 = phi - atan2(num, den)
		return alpha1



if (__name__ == "__main__"):
	arm = Arm()
	angles = [0, pi/4, -pi/4, -pi/2]
	[x, y, z] = arm.forwardKinematics(angles)
	# print x, y, z
	
	sol = arm.inverseKinematics([0, 50, 0], True, [0, 0, 0, 0])
	print array(sol) * 180 / pi
	print arm.forwardKinematics(sol)


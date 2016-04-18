from math import *
from math498 import *
from numpy import *
from ArmPlot import *

class PaperOrientation:
	VERTICAL = 0
	HORIZONTAL = 1

class Arm:
	def __init__(self):
		self.lengths = [13., 25.5, 26.2, 2.]
		self.angleOffsets = [0, 0.446, -pi/2 - 0.446, 0]

		self.tipLateralOffset = 0

		# offsets
		self.angle_lower_bounds = [-pi/2 + offset for offset in self.angleOffsets]
		self.angle_upper_bounds = [pi/2 + offset for offset in self.angleOffsets]

		# print("BOUNDS:")
		# print self.angle_lower_bounds
		# print self.angle_upper_bounds

		self.armPlot = ArmPlot([0, 0, 0, 0], self.lengths)

	def plot(self, angles, ax):
		kinematicAngles = self.armAnglesToKinematicAngles(angles)
		# print kinematicAnglesa
		self.armPlot.plotReal3D(kinematicAngles, ax)



	def inverseKinematics(self, pos, orientation):
		x = pos[0]
		y = pos[1]
		z = pos[2] - self.lengths[0]


		#TODO: change x, y, z depending on orientation

		uncorrectedR2 = sqrt(z*z + x*x + y*y)

		deltaTheta = atan2(self.tipLateralOffset, uncorrectedR2)

		# changing into spherical coordinates
		r1 = sqrt(x*x + y*y + self.tipLateralOffset*self.tipLateralOffset)
		r2 = sqrt(z*z + x*x + y*y + self.tipLateralOffset*self.tipLateralOffset)
		theta = atan2(y, x) - deltaTheta
		phi = asin(z / r2)





		d1 = self.lengths[1]
		d2 = self.lengths[2]


		print("R1: " + str(r1))
		print("Z: " + str(z))



		alpha1Sqrt = sqrt(-d1**6*z**2 + 
					2*d1**4*d2**2*z**2 +
					2*d1**4*r1**2*z**2+
					2*d1**4*z**4-
					d1**2*d2**4*z**2+
					2*d1**2*d2**2*r1**2*z**2+
					2*d1**2*d2**2*z**4-
					d1**2*r1**4*z**2-
					2*d1**2*r1**2*z**4-
					d1**2*z**6
				)

		alpha1Scalar = 1/(2*(d1**2*r1**2 + d2**2*z**2))

		alpha1Offset = d1*d2**2*r1 + d1*r1**3 + d1*r1*z**2
		alpha1Inner = alpha1Scalar * (d1**3*r1 - alpha1Sqrt - alpha1Offset)

		alpha1 = -acos(alpha1Inner)
		print("ALPHA1: " + str(alpha1*180/pi))

		alpha2Num = -d1**2-d2**2+r1**2+z**2
		alpha2Den = 2*d1*d2
		alpha2 = -acos(alpha2Num / alpha2Den)
		print("ALPHA2: " + str(alpha2 * 180/pi))




	# def armAnglesToKinematicAngles(self, armAngles):
	# 	# recieves arm angles in radians
	# 	angles = []
	# 	for angle, offset in zip(armAngles, self.angleOffsets):
	# 		newAngle = angle + offset
	# 		angles.append(newAngle)

	# 	return angles

	# def kinematicAnglesToArmAngles(self, kinematicAngles):
	# 	# recieves kinematic angles in radians 
	# 	angles = []
	# 	for angle, offset in zip(kinematicAngles, self.angleOffsets):
	# 		newAngle = angle - offset
	# 		angles.append(newAngle)

	# 	return angles
			

	# def forwardKinematics(self, armAngles):
	# 	# angles is in arm angles
	# 	angles = self.armAnglesToKinematicAngles(armAngles)

	# 	r1 = self.lengths[1] * cos(angles[1]) + \
	# 		self.lengths[2] * cos(angles[1] + angles[2]) + \
	# 		self.lengths[3] * cos(angles[1] + angles[2] + angles[3])

	# 	z = self.lengths[0] + \
	# 		self.lengths[1] * sin(angles[1]) + \
	# 		self.lengths[2] * sin(angles[1] + angles[2]) + \
	# 		self.lengths[3] * sin(angles[1] + angles[2] + angles[3])


	# 	deltaTheta = atan2(self.tipLateralOffset, r1)
	# 	correctedR1 = r1 * cos(deltaTheta)

	# 	x = correctedR1 * cos(angles[0] + deltaTheta)
	# 	y = correctedR1 * sin(angles[0] + deltaTheta)
	# 	return [x, y, z]


	# def inverseKinematics(self, pos, curr_arm_angles, orientation=PaperOrientation.HORIZONTAL):
	# 	curr_angles = self.armAnglesToKinematicAngles(curr_arm_angles)
		
	# 	x = pos[0]
	# 	y = pos[1]
	# 	z = pos[2] - self.lengths[0]

	# 	uncorrectedR2 = sqrt(z*z + x*x + y*y)

	# 	deltaTheta = atan2(self.tipLateralOffset, uncorrectedR2)

	# 	# changing into spherical coordinates
	# 	r2 = sqrt(z*z + x*x + y*y + self.tipLateralOffset*self.tipLateralOffset)
	# 	theta = atan2(y, x) - deltaTheta
	# 	phi = asin(z / r2)

	# 	if (orientation == PaperOrientation.VERTICAL):
	# 		end_effector_ang = 0
	# 	elif (orientation == PaperOrientation.HORIZONTAL):
	# 		end_effector_ang = -pi/2




	# 	minCost = -1
	# 	minCostSol = None
	# 	for alpha2 in linspace(self.angle_lower_bounds[2], self.angle_upper_bounds[2], 500):

	# 		A = -2*self.lengths[1]*self.lengths[3]*sin(alpha2)
	# 		B = 2*self.lengths[2]*self.lengths[3] + 2*self.lengths[1]*self.lengths[3]*cos(alpha2)
	# 		C = self.lengths[1]**2 + self.lengths[2]**2 + self.lengths[3]**2
			
	# 		det = (r2**2 - C - 2*self.lengths[1]*self.lengths[2]*cos(alpha2)) / sqrt(A**2 + B**2)
	# 		if (det < -1 or det > 1):
	# 			continue


	# 		sols = self.__inverseKinematicsHelper(alpha2, r2, phi, A, B, C, end_effector_ang)

	# 		sol1 = [sols[0][0], alpha2, sols[0][1]]
	# 		sol2 = [sols[1][0], alpha2, sols[1][1]]
	# 		solutions = [sol1, sol2]
	# 		for sol in solutions:
	# 			fullSol = [theta, sol[0], sol[1], sol[2]]
	# 			if (not self.__inverseKinematicsCheckValidity(fullSol, end_effector_ang)):
	# 				continue
	# 			cost = self.__inverseKinematicsCost(curr_angles, fullSol, end_effector_ang)
	# 			# temp = [theta, sol[0], sol[1], sol[2]]
	# 			# print temp
	# 			# temp = self.kinematicAnglesToArmAngles(temp)
	# 			# print self.forwardKinematics(temp)
	# 			if (minCost == -1 or cost < minCost):
	# 				minCost = cost
	# 				minCostSol = sol

	# 	if (minCostSol == None):
	# 		return None


	# 	kinematicSol = [theta, minCostSol[0], minCostSol[1], minCostSol[2]]
	# 	armSol = self.kinematicAnglesToArmAngles(kinematicSol)
	# 	armSol[3] = -armSol[0] 
	# 	if (orientation == PaperOrientation.VERTICAL):
	# 		armSol[3] += pi/2
	# 	return armSol


	# def __inverseKinematicsCheckValidity(self, angles, end_effector_ang):
	# 	for angle, lower, upper in zip(angles, self.angle_lower_bounds, self.angle_upper_bounds):
	# 		wrappedAngle = angle_wrap_pi(angle)
	# 		if (wrappedAngle < lower or wrappedAngle > upper):
	# 			return False

	# 	angleSum = angles[1] + angles[2] + angles[3]
	# 	angleSum = angle_wrap_pi(angleSum)

	# 	if (abs(angleSum - end_effector_ang) > 0.1):
	# 		return False
	# 	return True


	# def __inverseKinematicsCost(self, curr_angles, angles, targetEEAngle):


	# 	angleSum = angles[1] + angles[2] + angles[3]
	# 	angleSum = angle_wrap_pi(angleSum)

	# 	delta = angleSum - targetEEAngle

	# 	angleDiff = array(curr_angles) - array(angles)

	# 	return dot(angleDiff, angleDiff) + sqrt(delta * delta)



	# def __inverseKinematicsHelper(self, alpha2, r2, phi, A, B, C, targetEEAngle):
	# 	alpha3 = asin((r2**2-C-2*self.lengths[1]*self.lengths[2]*cos(alpha2)) / (sqrt(A**2 + B**2))) - atan2(B, A)

	# 	alpha3_2 = pi - alpha3 - 2*atan2(B, A)

	# 	alpha1 = self.__inverseKinematicsAlpha1(alpha2, alpha3, phi)
	# 	alpha1_2 = self.__inverseKinematicsAlpha1(alpha2, alpha3_2, phi)

	# 	alpha3_corrected = targetEEAngle - alpha2 - alpha1
	# 	alpha3_2_corrected = targetEEAngle - alpha2 - alpha1_2

	# 	return [[alpha1, alpha3_corrected], \
	# 		[alpha1_2, alpha3_2_corrected]]
		
	# def __inverseKinematicsAlpha1(self, alpha2, alpha3, phi):
	# 	# num = self.lengths[2]*sin(alpha2) + self.lengths[3]*sin(alpha2+alpha3)
	# 	# den = self.lengths[1] + self.lengths[2]*cos(alpha2) + self.lengths[3]*cos(alpha2+alpha3)

	# 	# alpha1 = phi - atan2(num, den)
	# 	# return alpha1
	# 	num = self.lengths[2]*sin(alpha2)
	# 	den = self.lengths[1] + self.lengths[2]*cos(alpha2)

	# 	alpha1 = phi - atan2(num, den)
	# 	return alpha1



if (__name__ == "__main__"):
	arm = Arm()
	# angles = [0, pi/4, -pi/4, -pi/2]
	# angles = [0, -0.1, 0, 0]
	# [x, y, z] = arm.forwardKinematics(angles)
	# print x, y, z

	sol = arm.inverseKinematics([0, 10, 20], PaperOrientation.VERTICAL)
	# print array(sol) * 180 / pi
	# print arm.forwardKinematics(sol)


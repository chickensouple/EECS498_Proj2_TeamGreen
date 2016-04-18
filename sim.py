from numpy import *
from numpy.linalg import *
from scipy.linalg import expm as expM
from matplotlib import *
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d import Axes3D
from ArmPlot import *
from Arm import *
from Coordinates import *
from Constants import *
import pdb

ion()
paperPoints = array([[0, -paperWidth/2, 0],
  [0, paperWidth/2, 0],
  [paperLength, paperWidth/2, 0],
  [paperLength, -paperWidth/2, 0],
  [0, -paperWidth/2, 0]])
paperPoints[:, 0] += 20 # x
paperPoints[:, 2] += 5 # z


a = Arm()
a.armPlot.setPaperPoints(paperPoints)
f = gcf()
f.clf()
ax = f.gca(projection='3d')


startAng = array([0, 0, 0, 0])

points = [[40, 0, 10], [42, 0, 10], [44, 0, 10], [46, 0, 10], [48, 0, 10]]

# endAng = array([0, 1, 0, 0])
# endAng = a.inverseKinematics([40, 0, 10], -pi/2, startAng)
# print endAng
# print a.armAnglesToKinematicAngles(endAng)
# # print a.forwardKinematics(endAng)


timeTaken = 1
pauseTime = 0.1
scalar = 1 / (timeTaken / pauseTime)
currAng = startAng

i = 0

while True:
  # ax.cla()
  a.plot(currAng, ax)
  ax.set(xlim=[-50,50],ylim=[-50,50],zlim=[-50,50])
  draw()
  if (len(points) != 0):
    currPos = a.forwardKinematics(currAng)
    dist = (currPos[0] - points[0][0])**2 + (currPos[1] - points[0][1])**2 + (currPos[2] - points[0][2])**2

    if (dist < 0.2):
      points.pop(0)
      continue

    print("CurrPos: " + str(currPos) + "\tTarget: " + str(points[0]))

    endAng = a.inverseKinematics(points[0], -pi/2, currAng)

    currAng = currAng + (endAng - currAng) * scalar

  # if (i < timeTaken / pauseTime):
  #   currAng = currAng + (endAng - startAng) * scalar
  #   i += 1
  pause(pauseTime)









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


# ion()
# a = Arm([0, 0, pi/2, 0], [1, -1, 1, 1])
# f = gcf()
# f.clf()
# ax = f.gca(projection='3d')


# coordinates = Coordinates()
# coordinates.calibrate(a.paperPoints[0:4, :])

# startAng = array([1.5, 0, 0, 1])
# currAng = startAng

# wayPts = [array([0, 2]), array([9, 8]), array([20, 22])]
# idx = 0

# while True:
#   ax.cla()
#   a.plotReal3D(currAng, ax)

#   if (idx >= len(wayPts)):
#     pause(0.1)
#     continue


#   currPos = a.getTool(currAng)
#   augmentedWaypt = array([wayPts[idx][0], wayPts[idx][1], 1])
#   targetPos = coordinates.transformPaperToReal(augmentedWaypt)

#   if (norm(targetPos - currPos[0:3]) < 0.1):
#     idx += 1
#     if (idx >= len(wayPts)):
#       continue
#     continue

#   direction = targetPos - currPos[0:3]
#   Jt = a.getToolJac(currAng)
#   currAng = currAng + 0.2 * dot(pinv(Jt)[:,:len(direction)],direction)
#   currAng[3] = a.calculateEndEffectorAngles(True, [currAng[1], currAng[2]])


#   pause(0.1)


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
endAng = array([0, 1, 0, 0])
endAng = a.inverseKinematics([40, 0, 10], -pi/2, startAng)
print endAng
print a.armAnglesToKinematicAngles(endAng)
# print a.forwardKinematics(endAng)


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
  if (i < timeTaken / pauseTime):
    currAng = currAng + (endAng - startAng) * scalar
    i += 1
  pause(pauseTime)









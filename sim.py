from numpy import *
from numpy.linalg import *
from scipy.linalg import expm as expM
from matplotlib import *
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d import Axes3D
from arm import *
from Coordinates import *
import pdb


ion()
a = Arm()
f = gcf()
f.clf()
ax = f.gca(projection='3d')

coordinates = Coordinates()
coordinates.calibrate(a.paperPoints[0:4, :])

startAng = array([1.5, 0, 0, 1])
currAng = startAng

wayPts = [array([0, 2]), array([9, 8]), array([20, 22])]
idx = 0

while True:

  ax.cla()
  a.plotReal3D(currAng, ax)

  if (idx >= len(wayPts)):
    pause(0.1)
    continue


  currPos = a.getTool(currAng)
  augmentedWaypt = array([wayPts[idx][0], wayPts[idx][1], 1])
  targetPos = coordinates.transformPaperToReal(augmentedWaypt)

  if (norm(targetPos - currPos[0:3]) < 0.1):
    idx += 1
    if (idx >= len(wayPts)):
      continue
    continue

  direction = targetPos - currPos[0:3]
  Jt = a.getToolJac(currAng)
  currAng = currAng + 0.2 * dot(pinv(Jt)[:,:len(direction)],direction)

  pause(0.1)




from numpy import *
from numpy.linalg import *
from scipy.linalg import expm as expM
from matplotlib import *
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d import Axes3D
from math498 import *
import pdb

def seToSE( x ):
  """
  Convert a twist (a rigid velocity, element of se(3)) to a rigid
  motion (an element of SE(3))
  
  INPUT:
    x -- 6 sequence
  OUTPUT:
    result -- 4 x 4  

  """
  x = asarray(x,dtype=float)
  if x.shape != (6,):
    raise ValueError("shape must be (6,); got %s" % str(x.shape))
  #
  return expM(screw(x))

def screw( v ):
  """
  Convert a 6-vector to a screw matrix 
  
  The function is vectorized, such that:
  INPUT:
    v -- N... x 6 -- input vectors
  OUTPUT:
    N... x 4 x 4  
  """
  v = asarray(v)
  z = zeros_like(v[0,...])
  return array([
      [ z, -v[...,5], v[...,4], v[...,0] ],
      [ v[...,5],  z,-v[...,3], v[...,1] ],
      [-v[...,4],  v[...,3], z, v[...,2] ],
      [ z,         z,        z, z] ])

class ArmPlot( object ):
  """
  class Arm
  
  Represents a series manipulator made of several segments.
  Each segment is graphically represented by a wireframe model
  """
  def __init__(self, angleOffsets, lengths):
    self.geom = [array([[0, 0, 0, 1]]).T]
    self.geom.append(array([[0, 0, 0, 1], [0, 0, lengths[0], 1]]).T)

    angleSum = 0
    for i in range(1, len(lengths)):
      lastGeom = self.geom[-1][:, -1]
      currLen = lengths[i]
      angleSum -= angleOffsets[i]
      geomDelta = array([currLen * cos(angleSum), 0, currLen * sin(angleSum), 0])
      temp = array([lastGeom, lastGeom + geomDelta]).T
      self.geom.append(temp)

    motorDirs = array([[0, 0, 1], [0, 1, 0], [0, 1, 0], [0, 1, 0]])

    tw = []
    for i in range(len(self.geom) - 1):
      w = motorDirs[i, :]
      currPos = self.geom[i][0:3, -1]
      v = -cross(w, currPos)
      tw.append( concatenate([v,w],0) )


    # Build an array of collected twists
    self.tw = asarray(tw)
    self.tool = self.geom[-1][:, -1]

    self.paperPoints = None
    self.toolHistory = None

    self.penOffset = pi / 4

  def setPaperPoints(self, paperPoints):
    self.paperPoints = concatenate([paperPoints, paperPoints[0, :][newaxis, :]], 0)

  def calculateEndEffectorAngles(self, vertical, angles):
    theta1 = angles[0]
    theta2 = angles[1]
    angle = -(pi/2) + (-theta1 + pi - theta2) + self.penOffset
    if (vertical):
      angle -= pi/2
    angle = angle_wrap_pi(angle)

    return angle


  def at( self, ang ):
    """
    Compute the rigid transformations for a multi-segment arm
    at the specified angles
    """
    ang = asarray(ang)[:,newaxis]
    tw = ang * self.tw
    A = [identity(4)]
    for twi in tw:
      M = seToSE(twi)
      A.append(dot(A[-1],M))
    return A
    
  def getTool( self, ang ):
    """
    Get "tool tip" position in world coordinates
    """
    # Get the rigid transformation for the last segment of the arm
    M = self.at(ang)[-1]
    return dot(M, self.tool)

  def plotReal3D(self, ang, ax):
    A = self.at(ang)
    for a,g in zip(A, self.geom):
      ng = dot(a, g)
      if ng.shape[1]<2:
        continue
      ax.plot3D(ng[0,:],ng[1,:],ng[2,:])



    tp = dot(a, self.tool)
    tp = tp[:, newaxis]



    if (self.toolHistory == None):
      self.toolHistory = tp
    else:
      self.toolHistory = concatenate([self.toolHistory, tp], 1)
      ax.plot3D(self.toolHistory[0, :], self.toolHistory[1, :], self.toolHistory[2, :])

    # # draw arena
    # ax.plot3D(self.arenaPoints[:, 0], self.arenaPoints[:, 1], self.arenaPoints[:, 2])

    # draw paper
    if (self.paperPoints != None):
      ax.plot3D(self.paperPoints[:, 0], self.paperPoints[:, 1], self.paperPoints[:, 2])

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')


if (__name__ == "__main__"):
  arm = Arm([0, -pi/5, pi/2, 0], [10, 30, 25, 5])



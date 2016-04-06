from joy import *
from numpy import *
from numpy.linalg import *

class MovePlan(Plan):
  def __init__(self, app, *arg, **kw):
    Plan.__init__(self, app, *arg, **kw)
    self.pos = None
    self.threshold = 0.1

  def setPos(self):
    self.pos = pos


  def behavior(self):
    angles = self.app.get_arm_angles()
    currPos = self.app.arm.getTool(angles)
    
    while (norm(currPos - pos) > self.threshold):
      direction = self.pos - currPos

      Jt = self.app.arm.getToolJac(ang)
      newAngles = angles + dot(pinv(Jt)[:,:len(direction)],direction)

      self.app.set_arm_angles(newAngles)
      yield self.forDuration(0.05)

      angles = self.app.get_angles()
      currPos = self.app.arm.getTool(angles)

    yield


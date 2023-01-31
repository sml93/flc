import numpy as np

""" Pump (ground) --> Nozzle (air) """

class bernoulli():
  """ class for calculating v2  from bernoulli's theorem """
  def __init__(self, d1, d2, h1, h2):
    self.rho = 997
    self.mdot = 660/3600
    self.Area1 = 0
    self.Area2 = 0
    self.d1 = d1
    self.d2 = d2
    self.v1 = 0
    self.v2 = 0
    self.p1 = 0
    self.p2 = 0
    self.h1 = h1
    self.h2 = h2
    self.fhl = 0
    self.g = 9.81

  def getArea(self, diameter):
    radius = diameter/2000
    return (np.pi*np.power(radius,2))

  def mdot2vel(self, diameter):
    area = self.getArea(diameter)
    return self.mdot/(self.rho*area)

  def getVelOut(self):
    """ Through Conservation of Mass (continuity) """
    self.v1 = self.mdot2vel(self.d1)
    self.Area1 = self.getArea(self.d1)
    self.Area2 = self.getArea(self.d2)
    self.v2 = ((self.Area1*self.v1)/self.Area2)
    print("\nvelIn: ", self.v1)
    print("\nvelOut: ", self.v2)
    print("\nAreaIn: ", self.Area1)
    print("\nAreaOut: ", self.Area2)
    return self.v2

  def frictionHead(self):
    """ Getting friction headloss """
    self.getVelOut()
    D = self.d2/1000  ## diameter of output
    lmbd = 0.02 + (1 / (2000*D))
    print("\nLambda: ", lmbd)
    L = self.h2/np.sin(np.deg2rad(45))  ## getting length of hose, L
    self.fhl = (lmbd*L*np.power(self.v2,2)*self.rho*self.g)/(2*D*self.g)
    print("\nFriction headloss: ", self.fhl)
    return self.fhl

  def getPumpPressure(self):
    """ p1 + 1/2*rho*v1^2 + rho*g*h1 + pp = p2 + 1/2*rho*v2^2 + rho*g*h2 + fhl """
    self.frictionHead()
    pp = (0.5*self.rho*np.power(self.v2,2)) + (self.rho*self.g*self.h2) + self.fhl
    print("\nPump pressure: ", pp)
    print("\nPump pressure(in bar): ", pp/10000)
    return pp

  def getForce(self):
    """ Jet force = rho*A*v*vjet + 0.5*rho*(v2^2-v1^2)*A """
    self.getPumpPressure()
    # Fjet = self.mdot*self.v2  ## We assume second term = 0 since P1 = P2 = Patm
    print(self.v2)
    Fjet = self.mdot*self.v2  + 0.5*self.rho*(np.power(self.v2,2)-np.power(self.v1,2))*self.Area2  
    Fjet = Fjet/9.81
    print("\nFjet (N): ", Fjet)
    print("\n Fjet (kgf): ", Fjet/9.81)
    return Fjet 

def main():
  run = bernoulli(9.4, 6.9, 0, 2.4)
  run.getForce()


if __name__ == "__main__":
  main()
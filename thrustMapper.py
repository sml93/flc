import numpy as np

from matplotlib import pyplot as plt

thrustSP = [0.1, 0.2, 0.3, 0.5, 0.75, 1.0]
thrust = [80, 200, 233, 360.333, 616, 711]

thrustSP2 = np.linspace(0.0,1.0,10)

def getXval(y, m, c, ce_i):
  print(y,m,c,ce_i)
  x = ((y-(ce_i))-c)/m
  print('x:', x)
  return x

def getThrustSP(value, ce=0):
  z = np.polyfit(thrustSP, thrust, 1)
  p = np.poly1d(z)
  ce_i = ce*1000/4
  print(p(value))
  print("ceiling effect (g): ", ce_i)
  print("thrust value (g): ", p(value))

  # coeff = (round(p[0],5), round(p[1],5), round(p[2],5)-(ce*1000/4))
  # # print(coeff)
  # roots = np.roots(coeff)
  # # print('roots: ', roots)
  # for i in range(len(roots)):
  #   if roots[i] > 0 and roots[i] < 1:
  #     print("\nroot: ", roots[i])

  diffthrustSP = getXval(p(value),p[1], p[0], ce_i)
  print('diff: ', diffthrustSP)
  newThrustSP = value - diffthrustSP
  print(newThrustSP)

  # plt.plot(thrustSP, thrust, '.')
  # plt.plot(thrustSP2, p(thrustSP2), '-')
  # plt.show()
  # return p(value)
  return newThrustSP

# getThrust(0.38)
getThrustSP(0.395,24.0)
# getThrust(0.9803)